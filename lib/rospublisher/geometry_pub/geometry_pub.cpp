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

REGISTER_FUNCTION(TwistPub);
REGISTER_FUNCTION(TwistVectPub);
REGISTER_FUNCTION(Twist2DPub);
REGISTER_FUNCTION(AccelPub);
REGISTER_FUNCTION(AccelVectPub);
REGISTER_FUNCTION(Accel2DPub);

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
	if( output.size() != 6 ) throw std::invalid_argument("TwistPub : Output dimension should be 6 !");
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
	if( output.size() != 6 ) throw std::invalid_argument("TwistVectPub : Output dimension should be 6 !");
	FMatrixPub<geometry_msgs::Twist>::setparameters();
        Kernel::iBind(lin,"lin", getUuid());
        Kernel::iBind(rot,"rot", getUuid());
}

void TwistVectPub::prerun()
{
	if( lin().iSize() != 3 )  throw std::invalid_argument("TwistVectPub : Linear Input dimension should be 3 !");
	if( rot().iSize() != 3 )  throw std::invalid_argument("TwistVectPub : Rotational Input dimension should be 3 !");

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
	if( output.size() != 2 ) throw std::invalid_argument("Twist2DPub : Output dimension should be 2 !");
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
        if(output.size() != 6) throw std::invalid_argument("AccelPub : Output dimension should be 6 !");
	
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
        if( output.size() != 6 ) throw std::invalid_argument("AccelVectPub : Output dimension should be 6 !");
	
	FMatrixPub<geometry_msgs::Accel>::setparameters();
        Kernel::iBind(lin,"lin", getUuid());
        Kernel::iBind(rot,"rot", getUuid());
}

void AccelVectPub::prerun()
{
        if( lin().iSize() != 3 )  throw std::invalid_argument("AccelVectPub : Linear Input dimension should be 3 !");
        if( rot().iSize() != 3 )  throw std::invalid_argument("AccelVectPub : Rotational Input dimension should be 3 !");
	
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
        if( output.size() != 2 ) throw std::invalid_argument("Accel2DPub : Output dimension should be 2 !");
	
	FMatrixPub<geometry_msgs::Accel>::setparameters();
        Kernel::iBind(lin,"lin", getUuid());
        Kernel::iBind(rot,"rot", getUuid());
}
