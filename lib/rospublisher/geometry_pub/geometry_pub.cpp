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


#include "geometry_pub.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

REGISTER_FUNCTION(Vector3Pub);
REGISTER_FUNCTION(Vector3SPub);
REGISTER_FUNCTION(TwistPub);
REGISTER_FUNCTION(TwistVectPub);
REGISTER_FUNCTION(Twist2DPub);
REGISTER_FUNCTION(AccelPub);
REGISTER_FUNCTION(AccelVectPub);
REGISTER_FUNCTION(Accel2DPub);
REGISTER_FUNCTION(PoseStampedPub);

/*******************************************************************************************************/
/********************                       Vector3SPub                            *********************/
/*******************************************************************************************************/

void Vector3SPub::compute()
{
	auto mout = getMapVect(output);
	mout[0] = x()();
	mout[1] = y()();
	mout[2] = z()();
	
	geometry_msgs::Vector3 msg;
	msg.x = mout[0];
	msg.y = mout[1];
	msg.z = mout[2];
}

void Vector3SPub::setparameters()
{
	if( output.size() != 3 ) throw std::invalid_argument("Vector3SPub : Output must be a 3D vector");

	FMatrixPub<geometry_msgs::Vector3>::setparameters();
 	
	Kernel::iBind(x,"x", getUuid());
	Kernel::iBind(y,"y", getUuid());
	Kernel::iBind(z,"z", getUuid());
}

/*******************************************************************************************************/
/********************                        Vector3Pub                            *********************/
/*******************************************************************************************************/

void Vector3Pub::compute()
{
	auto mout = getMapVect(output);
	mout = inVector()();
	
	geometry_msgs::Vector3 msg;
	msg.x = mout[0];
	msg.y = mout[1];
	msg.z = mout[2];
}

void Vector3Pub::prerun()
{
	if( inVector().iSize() != 3 || !inVector().isVect())  throw std::invalid_argument("Vector3Pub : inVector must be a 3D Vector.");
	FMatrixPub<geometry_msgs::Vector3>::prerun();
}

void Vector3Pub::setparameters()
{
	if( output.size() != 3 ) throw std::invalid_argument("Vector3SPub : Output must be a 3D vector");

	FMatrixPub<geometry_msgs::Vector3>::setparameters();
 	
	Kernel::iBind(inVector,"inVector", getUuid());
}

/*******************************************************************************************************/
/********************                         TwistPub                             *********************/
/*******************************************************************************************************/

void TwistPub::compute()
{
	auto mout = getMapVect(output);
	mout[0] = linX()();
	mout[1] = linY()();
	mout[2] = linZ()();
	mout[3] = rotX()();
	mout[4] = rotY()();
	mout[5] = rotZ()();

	geometry_msgs::Twist msg;
	msg.linear.x = mout[0];
	msg.linear.y = mout[1];
	msg.linear.z = mout[2];
	msg.angular.x  = mout[3];
	msg.angular.y  = mout[4];
	msg.angular.z  = mout[5];

	pub.publish(msg);
}

void TwistPub::setparameters()
{
	if( output.size() != 6  ) throw std::invalid_argument("TwistPub : Output must be a 6D vector");
	FMatrixPub<geometry_msgs::Twist>::setparameters();

 	Kernel::iBind(linX,"lin.x", getUuid());
 	Kernel::iBind(linY,"lin.y", getUuid());
 	Kernel::iBind(linZ,"lin.z", getUuid());
 	Kernel::iBind(rotX,"rot.x", getUuid());
 	Kernel::iBind(rotY,"rot.y", getUuid());
 	Kernel::iBind(rotZ,"rot.z", getUuid());
}

/*******************************************************************************************************/
/********************                         TwistVectPub                           *******************/
/*******************************************************************************************************/

void TwistVectPub::compute()
{
	auto mout = getMapVect(output);
	
	MATRIX tmpO = lin()(); 

	auto mtmp = getMapVect(tmpO);
	mout[0] = mtmp[0];
	mout[1] = mtmp[1];
	mout[2] = mtmp[2];

	tmpO = rot()();
	mtmp = getMapVect(tmpO);
	mout[3] = mtmp[0];
	mout[4] = mtmp[1];
	mout[5] = mtmp[2];

	geometry_msgs::Twist msg;
	msg.linear.x = mout[0];
	msg.linear.y = mout[1];
	msg.linear.z = mout[2];
	msg.angular.x  = mout[3];
	msg.angular.y  = mout[4];
	msg.angular.z  = mout[5];

	pub.publish(msg);
}

void TwistVectPub::setparameters()
{
	if( output.size() != 6 ) throw std::invalid_argument("TwistVectPub : Output must be a 6D vector");

	FMatrixPub<geometry_msgs::Twist>::setparameters();
	lin.setCheckSize(false);
	rot.setCheckSize(false);
        Kernel::iBind(lin,"lin", getUuid());
        Kernel::iBind(rot,"rot", getUuid());
}

void TwistVectPub::prerun()
{
	if( lin().iSize() != 3 || !lin().isVect())  throw std::invalid_argument("TwistVectPub : lin must be a 3D Vector.");
	if( rot().iSize() != 3 || !rot().isVect())  throw std::invalid_argument("TwistVectPub : rot must be a 3D Vector.");

	FMatrixPub<geometry_msgs::Twist>::prerun();
}

/*******************************************************************************************************/
/********************                          Twist2DPub                           ********************/
/*******************************************************************************************************/

void Twist2DPub::compute()
{
	auto mout = getMapVect(output);

        mout[0] = lin()();
        mout[1] = rot()();

        geometry_msgs::Twist msg;
        msg.linear.x = mout[0];
        msg.linear.y = 0;
        msg.linear.z = 0;
        msg.angular.x  = 0;
        msg.angular.y  = 0;
        msg.angular.z  = mout[1];

        pub.publish(msg);
}

void Twist2DPub::setparameters()
{
	if( output.size() != 2 ) throw std::invalid_argument("Twist2DPub : Output must be a 2D !");
	FMatrixPub<geometry_msgs::Twist>::setparameters();
        Kernel::iBind(lin,"lin", getUuid());
        Kernel::iBind(rot,"rot", getUuid());
}

/*******************************************************************************************************/
/*********************                         AccelPub                           **********************/
/*******************************************************************************************************/

void AccelPub::compute()
{
        auto mout = getMapVect(output);
        mout[0] = linX()();
        mout[1] = linY()();
        mout[2] = linZ()();
        mout[3] = rotX()();
        mout[4] = rotY()();
        mout[5] = rotZ()();

	geometry_msgs::Accel msg;
        msg.linear.x = mout[0];
        msg.linear.y = mout[1];
        msg.linear.z = mout[2];
        msg.angular.x  = mout[3];
        msg.angular.y  = mout[4];
        msg.angular.z  = mout[5];

        pub.publish(msg);
}

void AccelPub::setparameters()
{
	if( output.size() != 6 ) throw std::invalid_argument("AccelPub : Output must be a 6D vector");
	
	FMatrixPub<geometry_msgs::Accel>::setparameters();

        Kernel::iBind(linX,"lin.x", getUuid());
        Kernel::iBind(linY,"lin.y", getUuid());
        Kernel::iBind(linZ,"lin.z", getUuid());
        Kernel::iBind(rotX,"rot.x", getUuid());
        Kernel::iBind(rotY,"rot.y", getUuid());
        Kernel::iBind(rotZ,"rot.z", getUuid());
}

/*******************************************************************************************************/
/********************                        AccelVectPub                            *******************/
/*******************************************************************************************************/

void AccelVectPub::compute()
{
        auto mout = getMapVect(output);

        MATRIX tmpO = lin()();

        auto mtmp = getMapVect(tmpO);
        mout[0] = mtmp[0];
        mout[1] = mtmp[1];
        mout[2] = mtmp[2];

        tmpO = rot()();
        mtmp = getMapVect(tmpO);
        mout[3] = mtmp[0];
        mout[4] = mtmp[1];
        mout[5] = mtmp[2];

        geometry_msgs::Accel msg;
        msg.linear.x = mout[0];
        msg.linear.y = mout[1];
        msg.linear.z = mout[2];
        msg.angular.x  = mout[3];
        msg.angular.y  = mout[4];
        msg.angular.z  = mout[5];

        pub.publish(msg);
}

void AccelVectPub::setparameters()
{
	if( output.size() != 6 ) throw std::invalid_argument("AccelVectPub : Output must be a 6D vector");
	
	FMatrixPub<geometry_msgs::Accel>::setparameters();
	lin.setCheckSize(false);
	rot.setCheckSize(false);
        Kernel::iBind(lin,"lin", getUuid());
        Kernel::iBind(rot,"rot", getUuid());
}

void AccelVectPub::prerun()
{
	if( lin().iSize() != 3 || !lin().isVect())  throw std::invalid_argument("AccelVectPub : lin must be a 3D Vector.");
	if( rot().iSize() != 3 || !rot().isVect())  throw std::invalid_argument("AccelVectPub : rot must be a 3D Vector.");
	
	FMatrixPub<geometry_msgs::Accel>::prerun();
}

/*******************************************************************************************************/
/********************                          Accel2DPub                            *******************/
/*******************************************************************************************************/

void Accel2DPub::compute()
{
        auto mout = getMapVect(output);

        mout[0] = lin()();
        mout[1] = rot()();

        geometry_msgs::Accel msg;
        msg.linear.x = mout[0];
        msg.linear.y = 0;
        msg.linear.z = 0;
        msg.angular.x  = 0;
        msg.angular.y  = 0;
        msg.angular.z  = mout[1];

        pub.publish(msg);
}

void Accel2DPub::setparameters()
{
        if( output.size() != 2 ) throw std::invalid_argument("Accel2DPub : Output must be 2D Vector.");
	
	FMatrixPub<geometry_msgs::Accel>::setparameters();
        Kernel::iBind(lin,"lin", getUuid());
        Kernel::iBind(rot,"rot", getUuid());
}

/*******************************************************************************************************/
/********************                      PoseStampedPub                            *******************/
/*******************************************************************************************************/

void PoseStampedPub::compute()
{
        auto mout = getMapVect(output);

        MATRIX tmpO = position()();

        auto mtmp = getMapVect(tmpO);
        mout[0] = mtmp[0];
        mout[1] = mtmp[1];
        mout[2] = mtmp[2];

	msg.pose.position.x = mout[0];
	msg.pose.position.y = mout[1];
	msg.pose.position.z = mout[2];

        tmpO = orientation()();
        mtmp = getMapVect(tmpO);
        mout[3] = mtmp[0];
        mout[4] = mtmp[1];
        mout[5] = mtmp[2];
	
	tf2::Quaternion quat;
	quat.setRPY( mout[3], mout[4], mout[5] );

	msg.pose.orientation = tf2::toMsg(quat);

	pub.publish(msg);
}

void PoseStampedPub::setparameters()
{
	if( output.size() != 6 ) throw std::invalid_argument("PoseStampedPub : Output must be a 6D vector");

	FMatrixPub<geometry_msgs::PoseStamped>::setparameters();
	position.setCheckSize(false);
	orientation.setCheckSize(false);
        Kernel::iBind(frame_id,"frame_id", getUuid());
        Kernel::iBind(position,"position", getUuid());
        Kernel::iBind(orientation,"orientation", getUuid());
}

void PoseStampedPub::prerun()
{
	if( position().iSize() != 3 || !position().isVect()) throw std::invalid_argument("PoseStampedPub : position must be a 3D Vector.");
	if( orientation().iSize() != 3 || !orientation().isVect()) throw std::invalid_argument("PoseStampedPub : orientation must be a 3D Vector.");

	msg.header.frame_id = frame_id;

	FMatrixPub<geometry_msgs::PoseStamped>::prerun();
}
