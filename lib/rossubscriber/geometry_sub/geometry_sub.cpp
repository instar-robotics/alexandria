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

#include "geometry_sub.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


REGISTER_FUNCTION(Vector3Sub);
REGISTER_FUNCTION(Vector3XSub);
REGISTER_FUNCTION(Vector3YSub);
REGISTER_FUNCTION(Vector3ZSub);
REGISTER_FUNCTION(AccelSub);
REGISTER_FUNCTION(AccelLinearSub);
REGISTER_FUNCTION(AccelAngularSub);
REGISTER_FUNCTION(TwistSub);
REGISTER_FUNCTION(TwistLinearSub);
REGISTER_FUNCTION(TwistAngularSub);
REGISTER_FUNCTION(PoseStampedSub);

/*******************************************************************************************************/
/*********************                           Vector3                            ********************/
/*******************************************************************************************************/

void Vector3Sub::setparameters()
{
	FMatrixSub<geometry_msgs::Vector3>::setparameters();

	if( output.size() != 3 )  throw std::invalid_argument("Vector3Sub : Output must be 3D Vector.");
}

void Vector3Sub::callback(const geometry_msgs::Vector3::ConstPtr &msg)
{
	auto mout = getMapVect(output);
        mout[0] = msg->x;
        mout[1] = msg->y;
        mout[2] = msg->z;
}

void Vector3XSub::callback( const geometry_msgs::Vector3::ConstPtr &msg )
{
	output = msg->x;
}

void Vector3YSub::callback( const geometry_msgs::Vector3::ConstPtr &msg )
{
	output = msg->y;
}

void Vector3ZSub::callback( const geometry_msgs::Vector3::ConstPtr &msg )
{
	output = msg->z;
}

/*******************************************************************************************************/
/*********************                            Accel                             ********************/
/*******************************************************************************************************/

void AccelSub::setparameters()
{
	FMatrixSub<geometry_msgs::Accel>::setparameters();

	if(output.size() != 6) throw std::invalid_argument("AccelSub : Output must be a 6D Vector.");
}

void AccelSub::callback(const geometry_msgs::Accel::ConstPtr &msg)
{
	auto mout = getMapVect(output);
	mout[0] =  msg->linear.x;
	mout[1] =  msg->linear.y;
	mout[2] =  msg->linear.z;
	mout[3] =  msg->angular.x;
	mout[4] =  msg->angular.y;
	mout[5] =  msg->angular.z;
}

void AccelLinearSub::setparameters()
{
        FMatrixSub<geometry_msgs::Accel>::setparameters();

        if(output.size() != 3) throw std::invalid_argument("AccelLinearSub : Output must be 3D Vector.");
}

void AccelLinearSub::callback(const geometry_msgs::Accel::ConstPtr &msg)
{
	auto mout = getMapVect(output);
	mout[0] =  msg->linear.x;
	mout[1] =  msg->linear.y;
	mout[2] =  msg->linear.z;
}


void AccelAngularSub::setparameters()
{
	FMatrixSub<geometry_msgs::Accel>::setparameters();

	if(output.size() != 3) throw std::invalid_argument("AccelAngularSub : Output must be 3D Vector.");
}

void AccelAngularSub::callback(const geometry_msgs::Accel::ConstPtr &msg)
{
	auto mout = getMapVect(output);
	mout[0] =  msg->angular.x;
	mout[1] =  msg->angular.y;
	mout[2] =  msg->angular.z;
}

/*******************************************************************************************************/
/*********************                            Twist                             ********************/
/*******************************************************************************************************/

void TwistSub::setparameters()
{
        FMatrixSub<geometry_msgs::Twist>::setparameters();

        if(output.size() != 6) throw std::invalid_argument("TwistSub : Output must be a 6D Vector.");
}

void TwistSub::callback(const geometry_msgs::Twist::ConstPtr &msg)
{
	auto mout = getMapVect(output);
	mout[0] =  msg->linear.x;
	mout[1] =  msg->linear.y;
	mout[2] =  msg->linear.z;
	mout[3] =  msg->angular.x;
	mout[4] =  msg->angular.y;
	mout[5] =  msg->angular.z;
}

void TwistLinearSub::setparameters()
{
        FMatrixSub<geometry_msgs::Twist>::setparameters();

        if(output.size() != 3) throw std::invalid_argument("TwistLinearSub : Output must be 3D Vector.");
}

void TwistLinearSub::callback(const geometry_msgs::Twist::ConstPtr &msg)
{
        auto mout = getMapVect(output);
        mout[0] =  msg->linear.x;
        mout[1] =  msg->linear.y;
        mout[2] =  msg->linear.z;
}

void TwistAngularSub::setparameters()
{
        FMatrixSub<geometry_msgs::Twist>::setparameters();

        if(output.size() != 3) throw std::invalid_argument("TwistAngularSub : Output must be 3D Vector.");
}

void TwistAngularSub::callback(const geometry_msgs::Twist::ConstPtr &msg)
{
        auto mout = getMapVect(output);
        mout[0] =  msg->angular.x;
        mout[1] =  msg->angular.y;
        mout[2] =  msg->angular.z;
}

/*******************************************************************************************************/
/*********************                          PoseStamped                         ********************/
/*******************************************************************************************************/


void PoseStampedSub::setparameters()
{
        FMatrixSub<geometry_msgs::PoseStamped>::setparameters();

        if(output.size() != 6) throw std::invalid_argument("PoseStampedSub : Output must be a 6D Vector.");
}

void PoseStampedSub::callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
        auto mout = getMapVect(output);
        mout[0] =  msg->pose.position.x;
        mout[1] =  msg->pose.position.y;
        mout[2] =  msg->pose.position.x;

	tf2::Quaternion quat_tf;
	tf2::convert(msg->pose.orientation , quat_tf);
	tf2::Matrix3x3 m(quat_tf);
	m.getRPY(mout[3], mout[4], mout[5]);
}

