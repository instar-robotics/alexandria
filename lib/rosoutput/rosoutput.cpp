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


#include "rosoutput.h"

REGISTER_FUNCTION(CmdVelRawOutput);
REGISTER_FUNCTION(CmdVelVectOutput);
REGISTER_FUNCTION(CmdVel2DOutput);

/*******************************************************************************************************/
/*****************                         CmdVelRawOutput                           *******************/
/*******************************************************************************************************/

void CmdVelRawOutput::compute()
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

void CmdVelRawOutput::setparameters()
{
	if( output.rows() * output.cols() != 6 ) throw std::invalid_argument("CmdVelRawOutput : Output dimension should be 6 !");

 	Kernel::iBind(topic_name,"topic_name", getUuid());
 	Kernel::iBind(size_queue,"size_queue", getUuid());
 	Kernel::iBind(linX,"lin.x", getUuid());
 	Kernel::iBind(linY,"lin.y", getUuid());
 	Kernel::iBind(linZ,"lin.z", getUuid());
 	Kernel::iBind(rotX,"rot.x", getUuid());
 	Kernel::iBind(rotY,"rot.y", getUuid());
 	Kernel::iBind(rotZ,"rot.z", getUuid());
}

void CmdVelRawOutput::prerun()
{
	ros::NodeHandle n;
	pub = n.advertise<geometry_msgs::Twist>( topic_name , size_queue()());
}


/*******************************************************************************************************/
/*****************                        CmdVelVectOutput                           *******************/
/*******************************************************************************************************/

void CmdVelVectOutput::compute()
{
	auto mout = getMapVect(output);
	
	MatrixXd tmpO = lin()(); 

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

void CmdVelVectOutput::setparameters()
{
	if( output.rows() * output.cols() != 6 ) throw std::invalid_argument("CmdVelRawOutput : Output dimension should be 6 !");

        Kernel::iBind(topic_name,"topic_name", getUuid());
 	Kernel::iBind(size_queue,"size_queue", getUuid());
        Kernel::iBind(lin,"lin", getUuid());
        Kernel::iBind(rot,"rot", getUuid());
}

void CmdVelVectOutput::prerun()
{
	if( lin().iSize() != 3 )  throw std::invalid_argument("CmdVelRawOutput : Linear Input dimension should be 3 !");
	if( rot().iSize() != 3 )  throw std::invalid_argument("CmdVelRawOutput : Rotational Input dimension should be 3 !");

	ros::NodeHandle n;
	pub = n.advertise<geometry_msgs::Twist>( topic_name , size_queue()());
}

/*******************************************************************************************************/
/*****************                          CmdVel2DOutput                           *******************/
/*******************************************************************************************************/

void CmdVel2DOutput::compute()
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

void CmdVel2DOutput::setparameters()
{
	if( output.rows() * output.cols() != 2 ) throw std::invalid_argument("CmdVelRawOutput : Output dimension should be 2 !");

        Kernel::iBind(topic_name,"topic_name", getUuid());
 	Kernel::iBind(size_queue,"size_queue", getUuid());
        Kernel::iBind(lin,"lin", getUuid());
        Kernel::iBind(rot,"rot", getUuid());
}

void CmdVel2DOutput::prerun()
{
	ros::NodeHandle n;
	pub = n.advertise<geometry_msgs::Twist>( topic_name , size_queue()());
}


REGISTER_FUNCTION(AccelRawOutput);
REGISTER_FUNCTION(AccelVectOutput);
REGISTER_FUNCTION(Accel2DOutput);

/*******************************************************************************************************/
/******************                         AccelRawOutput                           *******************/
/*******************************************************************************************************/

void AccelRawOutput::compute()
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

void AccelRawOutput::setparameters()
{
        if( output.rows() * output.cols() != 6 ) throw std::invalid_argument("AccelRawOutput : Output dimension should be 6 !");

        Kernel::iBind(topic_name,"topic_name", getUuid());
        Kernel::iBind(size_queue,"size_queue", getUuid());
        Kernel::iBind(linX,"lin.x", getUuid());
        Kernel::iBind(linY,"lin.y", getUuid());
        Kernel::iBind(linZ,"lin.z", getUuid());
        Kernel::iBind(rotX,"rot.x", getUuid());
        Kernel::iBind(rotY,"rot.y", getUuid());
        Kernel::iBind(rotZ,"rot.z", getUuid());
}

void AccelRawOutput::prerun()
{
        ros::NodeHandle n;
        pub = n.advertise<geometry_msgs::Accel>( topic_name , size_queue()());
}

/*******************************************************************************************************/
/******************                        AccelVectOutput                           *******************/
/*******************************************************************************************************/

void AccelVectOutput::compute()
{
        auto mout = getMapVect(output);

        MatrixXd tmpO = lin()();

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

void AccelVectOutput::setparameters()
{
        if( output.rows() * output.cols() != 6 ) throw std::invalid_argument("AccelRawOutput : Output dimension should be 6 !");

        Kernel::iBind(topic_name,"topic_name", getUuid());
        Kernel::iBind(size_queue,"size_queue", getUuid());
        Kernel::iBind(lin,"lin", getUuid());
        Kernel::iBind(rot,"rot", getUuid());
}

void AccelVectOutput::prerun()
{
        if( lin().iSize() != 3 )  throw std::invalid_argument("AccelRawOutput : Linear Input dimension should be 3 !");
        if( rot().iSize() != 3 )  throw std::invalid_argument("AccelRawOutput : Rotational Input dimension should be 3 !");

        ros::NodeHandle n;
        pub = n.advertise<geometry_msgs::Accel>( topic_name , size_queue()());
}

/*******************************************************************************************************/
/******************                          Accel2DOutput                           *******************/
/*******************************************************************************************************/

void Accel2DOutput::compute()
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

void Accel2DOutput::setparameters()
{
        if( output.rows() * output.cols() != 2 ) throw std::invalid_argument("AccelRawOutput : Output dimension should be 2 !");

        Kernel::iBind(topic_name,"topic_name", getUuid());
        Kernel::iBind(size_queue,"size_queue", getUuid());
        Kernel::iBind(lin,"lin", getUuid());
        Kernel::iBind(rot,"rot", getUuid());
}

void Accel2DOutput::prerun()
{
        ros::NodeHandle n;
        pub = n.advertise<geometry_msgs::Accel>( topic_name , size_queue()());
}

