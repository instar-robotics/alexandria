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

#include "nav.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

REGISTER_FUNCTION(OdoPos);
REGISTER_FUNCTION(OdoPosX);
REGISTER_FUNCTION(OdoPosY);
REGISTER_FUNCTION(OdoPosZ);
REGISTER_FUNCTION(OdoEuler);
REGISTER_FUNCTION(OdoEulerRoll);
REGISTER_FUNCTION(OdoEulerPitch);
REGISTER_FUNCTION(OdoEulerYaw);
REGISTER_FUNCTION(OdoQuater);
REGISTER_FUNCTION(OdoQuaterX);
REGISTER_FUNCTION(OdoQuaterY);
REGISTER_FUNCTION(OdoQuaterZ);
REGISTER_FUNCTION(OdoQuaterW);
REGISTER_FUNCTION(OdoTwistLin);
REGISTER_FUNCTION(OdoTwistLinX);
REGISTER_FUNCTION(OdoTwistLinY);
REGISTER_FUNCTION(OdoTwistLinZ);
REGISTER_FUNCTION(OdoTwistAng);
REGISTER_FUNCTION(OdoTwistAngRoll);
REGISTER_FUNCTION(OdoTwistAngPitch);
REGISTER_FUNCTION(OdoTwistAngYaw);

/*******************************************************************************************************/
/*********************                            OdoPos                            ********************/
/*******************************************************************************************************/


void OdoPos::compute()
{
        callOne(sleep()());
}

void OdoPos::setparameters()
{
        Kernel::iBind(topic_name,"topic_name", getUuid());
        Kernel::iBind(size_queue,"size_queue", getUuid());
        Kernel::iBind(sleep,"sleep", getUuid());
}

void OdoPos::prerun()
{
        if( output.rows() * output.cols() != 3 ) throw std::invalid_argument("OdoPos : Output dimension should be 3 !");
}

void OdoPos::callback(const nav_msgs::Odometry::ConstPtr &msg )
{
        auto mout = getMapVect(output);
        mout[0] = msg->pose.pose.position.x;
        mout[1] = msg->pose.pose.position.y;
        mout[2] = msg->pose.pose.position.z;
}

void OdoPos::onQuit()
{
        disable();
}

void OdoPos::onPause()
{
        disable();
}

void OdoPos::onRun()
{
        enable( topic_name, (int)(size_queue()())   );
}

/*******************************************************************************************************/
/*********************                           OdoPosX                            ********************/
/*******************************************************************************************************/


void OdoPosX::compute()
{
        callOne(sleep()());
}

void OdoPosX::setparameters()
{
        Kernel::iBind(topic_name,"topic_name", getUuid());
        Kernel::iBind(size_queue,"size_queue", getUuid());
        Kernel::iBind(sleep,"sleep", getUuid());
}

void OdoPosX::callback(const nav_msgs::Odometry::ConstPtr &msg )
{
        output = msg->pose.pose.position.x;
}

void OdoPosX::onQuit()
{
        disable();
}

void OdoPosX::onPause()
{
        disable();
}

void OdoPosX::onRun()
{
        enable( topic_name, (int)(size_queue()())   );
}

/*******************************************************************************************************/
/*********************                           OdoPosY                            ********************/
/*******************************************************************************************************/


void OdoPosY::compute()
{
        callOne(sleep()());
}

void OdoPosY::setparameters()
{
        Kernel::iBind(topic_name,"topic_name", getUuid());
        Kernel::iBind(size_queue,"size_queue", getUuid());
        Kernel::iBind(sleep,"sleep", getUuid());
}

void OdoPosY::callback(const nav_msgs::Odometry::ConstPtr &msg )
{
        output = msg->pose.pose.position.y;
}

void OdoPosY::onQuit()
{
        disable();
}

void OdoPosY::onPause()
{
        disable();
}

void OdoPosY::onRun()
{
        enable( topic_name, (int)(size_queue()())   );
}


/*******************************************************************************************************/
/*********************                           OdoPosZ                            ********************/
/*******************************************************************************************************/

void OdoPosZ::compute()
{
        callOne(sleep()());
}

void OdoPosZ::setparameters()
{
        Kernel::iBind(topic_name,"topic_name", getUuid());
        Kernel::iBind(size_queue,"size_queue", getUuid());
        Kernel::iBind(sleep,"sleep", getUuid());
}

void OdoPosZ::callback(const nav_msgs::Odometry::ConstPtr &msg )
{
        output = msg->pose.pose.position.z;
}

void OdoPosZ::onQuit()
{
        disable();
}

void OdoPosZ::onPause()
{
        disable();
}

void OdoPosZ::onRun()
{
        enable( topic_name, (int)(size_queue()())   );
}


/*******************************************************************************************************/
/*********************                          OdoEuler                            ********************/
/*******************************************************************************************************/

void OdoEuler::compute()
{
        callOne(sleep()());
}

void OdoEuler::setparameters()
{
        Kernel::iBind(topic_name,"topic_name", getUuid());
        Kernel::iBind(size_queue,"size_queue", getUuid());
        Kernel::iBind(sleep,"sleep", getUuid());
}

void OdoEuler::prerun()
{
        if( output.rows() * output.cols() != 3 ) throw std::invalid_argument("OdoEuler : Output dimension should be 3 !");
}

void OdoEuler::callback(const nav_msgs::Odometry::ConstPtr &msg )
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

void OdoEuler::onQuit()
{
        disable();
}

void OdoEuler::onPause()
{
        disable();
}

void OdoEuler::onRun()
{
        enable( topic_name, (int)(size_queue()())   );
}


/*******************************************************************************************************/
/*********************                       OdoEulerRoll                           ********************/
/*******************************************************************************************************/

void OdoEulerRoll::compute()
{
        callOne(sleep()());
}

void OdoEulerRoll::setparameters()
{
        Kernel::iBind(topic_name,"topic_name", getUuid());
        Kernel::iBind(size_queue,"size_queue", getUuid());
        Kernel::iBind(sleep,"sleep", getUuid());
}

void OdoEulerRoll::callback(const nav_msgs::Odometry::ConstPtr &msg )
{
        tf2::Quaternion q(msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z,msg->pose.pose.orientation.w);

        tf2::Matrix3x3 m(q);
        SCALAR roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        output = roll;
}

void OdoEulerRoll::onQuit()
{
        disable();
}

void OdoEulerRoll::onPause()
{
        disable();
}

void OdoEulerRoll::onRun()
{
        enable( topic_name, (int)(size_queue()())   );
}

/*******************************************************************************************************/
/*********************                      OdoEulerPitch                           ********************/
/*******************************************************************************************************/

void OdoEulerPitch::compute()
{
        callOne(sleep()());
}

void OdoEulerPitch::setparameters()
{
        Kernel::iBind(topic_name,"topic_name", getUuid());
        Kernel::iBind(size_queue,"size_queue", getUuid());
        Kernel::iBind(sleep,"sleep", getUuid());
}

void OdoEulerPitch::callback(const nav_msgs::Odometry::ConstPtr &msg )
{
        tf2::Quaternion q(msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z,msg->pose.pose.orientation.w);

        tf2::Matrix3x3 m(q);
        SCALAR roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        output = pitch;
}

void OdoEulerPitch::onQuit()
{
        disable();
}

void OdoEulerPitch::onPause()
{
        disable();
}

void OdoEulerPitch::onRun()
{
        enable( topic_name, (int)(size_queue()())   );
}


/*******************************************************************************************************/
/*********************                        OdoEulerYaw                           ********************/
/*******************************************************************************************************/

void OdoEulerYaw::compute()
{
        callOne(sleep()());
}

void OdoEulerYaw::setparameters()
{
        Kernel::iBind(topic_name,"topic_name", getUuid());
        Kernel::iBind(size_queue,"size_queue", getUuid());
        Kernel::iBind(sleep,"sleep", getUuid());
}

void OdoEulerYaw::callback(const nav_msgs::Odometry::ConstPtr &msg )
{
        tf2::Quaternion q(msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z,msg->pose.pose.orientation.w);

        tf2::Matrix3x3 m(q);
        SCALAR roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        output = yaw;
}

void OdoEulerYaw::onQuit()
{
        disable();
}

void OdoEulerYaw::onPause()
{
        disable();
}

void OdoEulerYaw::onRun()
{
        enable( topic_name, (int)(size_queue()())   );
}


/*******************************************************************************************************/
/*********************                         OdoQuater                            ********************/
/*******************************************************************************************************/

void OdoQuater::compute()
{
        callOne(sleep()());
}

void OdoQuater::setparameters()
{
        Kernel::iBind(topic_name,"topic_name", getUuid());
        Kernel::iBind(size_queue,"size_queue", getUuid());
        Kernel::iBind(sleep,"sleep", getUuid());
}

void OdoQuater::prerun()
{
        if( output.rows() * output.cols() != 4 ) throw std::invalid_argument("OdoQuater : Output dimension should be 3 !");
}

void OdoQuater::callback( const nav_msgs::Odometry::ConstPtr &msg )
{
        auto mout = getMapVect(output);
        mout[0] = msg->pose.pose.orientation.x;
        mout[1] = msg->pose.pose.orientation.y;
        mout[2] = msg->pose.pose.orientation.z;
        mout[3] = msg->pose.pose.orientation.w;
}

void OdoQuater::onQuit()
{
        disable();
}

void OdoQuater::onPause()
{
        disable();
}

void OdoQuater::onRun()
{
        enable( topic_name, (int)(size_queue()())   );
}

/*******************************************************************************************************/
/*********************                         OdoQuaterX                           ********************/
/*******************************************************************************************************/

void OdoQuaterX::compute()
{
        callOne(sleep()());
}

void OdoQuaterX::setparameters()
{
        Kernel::iBind(topic_name,"topic_name", getUuid());
        Kernel::iBind(size_queue,"size_queue", getUuid());
        Kernel::iBind(sleep,"sleep", getUuid());
}

void OdoQuaterX::callback(const nav_msgs::Odometry::ConstPtr &msg )
{
        output = msg->pose.pose.orientation.x;
}

void OdoQuaterX::onQuit()
{
        disable();
}

void OdoQuaterX::onPause()
{
        disable();
}

void OdoQuaterX::onRun()
{
        enable( topic_name, (int)(size_queue()())   );
}


/*******************************************************************************************************/
/*********************                         OdoQuaterY                           ********************/
/*******************************************************************************************************/

void OdoQuaterY::compute()
{
        my_queue.callOne(ros::WallDuration( sleep()() ));
}

void OdoQuaterY::setparameters()
{
        Kernel::iBind(topic_name,"topic_name", getUuid());
        Kernel::iBind(size_queue,"size_queue", getUuid());
        Kernel::iBind(sleep,"sleep", getUuid());
}

void OdoQuaterY::callback( const nav_msgs::Odometry::ConstPtr &msg )
{
        output = msg->pose.pose.orientation.y;
}

void OdoQuaterY::onQuit()
{
        disable();
}

void OdoQuaterY::onPause()
{
        disable();
}

void OdoQuaterY::onRun()
{
        enable( topic_name, (int)(size_queue()())   );
}

/*******************************************************************************************************/
/*********************                         OdoQuaterZ                           ********************/
/*******************************************************************************************************/

void OdoQuaterZ::compute()
{
        callOne(sleep()());
}

void OdoQuaterZ::setparameters()
{
        Kernel::iBind(topic_name,"topic_name", getUuid());
        Kernel::iBind(size_queue,"size_queue", getUuid());
        Kernel::iBind(sleep,"sleep", getUuid());
}

void OdoQuaterZ::callback( const nav_msgs::Odometry::ConstPtr &msg )
{
        output = msg->pose.pose.orientation.z;
}

void OdoQuaterZ::onQuit()
{
        disable();
}

void OdoQuaterZ::onPause()
{
        disable();
}

void OdoQuaterZ::onRun()
{
        enable( topic_name, (int)(size_queue()())   );
}

/*******************************************************************************************************/
/*********************                         OdoQuaterW                           ********************/
/*******************************************************************************************************/

void OdoQuaterW::compute()
{
        callOne(sleep()());
}

void OdoQuaterW::setparameters()
{
        Kernel::iBind(topic_name,"topic_name", getUuid());
        Kernel::iBind(size_queue,"size_queue", getUuid());
        Kernel::iBind(sleep,"sleep", getUuid());
}

void OdoQuaterW::callback(const nav_msgs::Odometry::ConstPtr &msg )
{
        output = msg->pose.pose.orientation.w;
}

void OdoQuaterW::onQuit()
{
        disable();
}

void OdoQuaterW::onPause()
{
        disable();
}

void OdoQuaterW::onRun()
{
        enable( topic_name, (int)(size_queue()())   );
}

/*******************************************************************************************************/
/*********************                        OdoTwistLin                           ********************/
/*******************************************************************************************************/

void OdoTwistLin::compute()
{
        callOne(sleep()());
}

void OdoTwistLin::setparameters()
{
        Kernel::iBind(topic_name,"topic_name", getUuid());
        Kernel::iBind(size_queue,"size_queue", getUuid());
        Kernel::iBind(sleep,"sleep", getUuid());
}

void OdoTwistLin::prerun()
{
        if( output.rows() * output.cols() != 3 ) throw std::invalid_argument("OdoTwistLin : Output dimension should be 3 !");
}

void OdoTwistLin::callback(const nav_msgs::Odometry::ConstPtr &msg )
{
        auto mout = getMapVect(output);
        mout[0] = msg->twist.twist.linear.x;
        mout[1] = msg->twist.twist.linear.y;
        mout[2] = msg->twist.twist.linear.z;
}

void OdoTwistLin::onQuit()
{
        disable();
}

void OdoTwistLin::onPause()
{
        disable();
}

void OdoTwistLin::onRun()
{
        enable( topic_name, (int)(size_queue()())   );
}



/*******************************************************************************************************/
/*********************                      OdoTwistLinX                            ********************/
/*******************************************************************************************************/

void OdoTwistLinX::compute()
{
        callOne(sleep()());
}

void OdoTwistLinX::setparameters()
{
        Kernel::iBind(topic_name,"topic_name", getUuid());
        Kernel::iBind(size_queue,"size_queue", getUuid());
        Kernel::iBind(sleep,"sleep", getUuid());
}

void OdoTwistLinX::callback(const nav_msgs::Odometry::ConstPtr &msg )
{
        output = msg->twist.twist.linear.x;
}

void OdoTwistLinX::onQuit()
{
        disable();
}

void OdoTwistLinX::onPause()
{
        disable();
}

void OdoTwistLinX::onRun()
{
        enable( topic_name, (int)(size_queue()())   );
}


/*******************************************************************************************************/
/*********************                       OdoTwistLinY                           ********************/
/*******************************************************************************************************/

void OdoTwistLinY::compute()
{
        callOne(sleep()());
}

void OdoTwistLinY::setparameters()
{
        Kernel::iBind(topic_name,"topic_name", getUuid());
        Kernel::iBind(size_queue,"size_queue", getUuid());
        Kernel::iBind(sleep,"sleep", getUuid());
}

void OdoTwistLinY::callback(const nav_msgs::Odometry::ConstPtr &msg )
{
        output = msg->twist.twist.linear.y;
}


void OdoTwistLinY::onQuit()
{
        disable();
}

void OdoTwistLinY::onPause()
{
        disable();
}

void OdoTwistLinY::onRun()
{
        enable( topic_name, (int)(size_queue()())   );
}

/*******************************************************************************************************/
/*********************                      OdoTwistLinZ                            ********************/
/*******************************************************************************************************/

void OdoTwistLinZ::compute()
{
        callOne(sleep()());
}

void OdoTwistLinZ::setparameters()
{
        Kernel::iBind(topic_name,"topic_name", getUuid());
        Kernel::iBind(size_queue,"size_queue", getUuid());
        Kernel::iBind(sleep,"sleep", getUuid());
}

void OdoTwistLinZ::callback(const nav_msgs::Odometry::ConstPtr &msg )
{
        output = msg->twist.twist.linear.z;
}

void OdoTwistLinZ::onQuit()
{
        disable();
}

void OdoTwistLinZ::onPause()
{
        disable();
}

void OdoTwistLinZ::onRun()
{
        enable( topic_name, (int)(size_queue()())   );
}

/*******************************************************************************************************/
/*********************                       OdoTwistAng                            ********************/
/*******************************************************************************************************/

void OdoTwistAng::compute()
{
        callOne(sleep()());
}

void OdoTwistAng::setparameters()
{
        Kernel::iBind(topic_name,"topic_name", getUuid());
        Kernel::iBind(size_queue,"size_queue", getUuid());
        Kernel::iBind(sleep,"sleep", getUuid());
}

void OdoTwistAng::prerun()
{
        if( output.rows() * output.cols() != 3 ) throw std::invalid_argument("OdoTwistAng : Output dimension should be 3 !");
}

void OdoTwistAng::callback(const nav_msgs::Odometry::ConstPtr &msg )
{
        auto mout = getMapVect(output);
        mout[0] = msg->twist.twist.angular.x;
        mout[1] = msg->twist.twist.angular.y;
        mout[2] = msg->twist.twist.angular.z;
}

void OdoTwistAng::onQuit()
{
        disable();
}

void OdoTwistAng::onPause()
{
        disable();
}

void OdoTwistAng::onRun()
{
        enable( topic_name, (int)(size_queue()())   );
}

/*******************************************************************************************************/
/*********************                     OdoTwistAngRoll                          ********************/
/*******************************************************************************************************/

void OdoTwistAngRoll::compute()
{
        callOne(sleep()());
}

void OdoTwistAngRoll::setparameters()
{
        Kernel::iBind(topic_name,"topic_name", getUuid());
        Kernel::iBind(size_queue,"size_queue", getUuid());
        Kernel::iBind(sleep,"sleep", getUuid());
}

void OdoTwistAngRoll::callback(const nav_msgs::Odometry::ConstPtr &msg )
{
        output = msg->twist.twist.angular.x;
}

void OdoTwistAngRoll::onQuit()
{
        disable();
}

void OdoTwistAngRoll::onPause()
{
        disable();
}

void OdoTwistAngRoll::onRun()
{
        enable( topic_name, (int)(size_queue()())   );
}

/*******************************************************************************************************/
/***********************                   OdoTwistAngPitch                       **********************/
/*******************************************************************************************************/

void OdoTwistAngPitch::compute()
{
        callOne(sleep()());
}

void OdoTwistAngPitch::setparameters()
{
        Kernel::iBind(topic_name,"topic_name", getUuid());
        Kernel::iBind(size_queue,"size_queue", getUuid());
        Kernel::iBind(sleep,"sleep", getUuid());
}

void OdoTwistAngPitch::callback(const nav_msgs::Odometry::ConstPtr &msg )
{
        output = msg->twist.twist.angular.y;
}

void OdoTwistAngPitch::onQuit()
{
        disable();
}

void OdoTwistAngPitch::onPause()
{
        disable();
}

void OdoTwistAngPitch::onRun()
{
        enable( topic_name, (int)(size_queue()())   );
}


/*******************************************************************************************************/
/*********************                       OdoTwistAngYaw                         ********************/
/*******************************************************************************************************/

void OdoTwistAngYaw::compute()
{
        callOne(sleep()());
}

void OdoTwistAngYaw::setparameters()
{
        Kernel::iBind(topic_name,"topic_name", getUuid());
        Kernel::iBind(size_queue,"size_queue", getUuid());
        Kernel::iBind(sleep,"sleep", getUuid());
}

void OdoTwistAngYaw::callback(const nav_msgs::Odometry::ConstPtr &msg )
{
        output = msg->twist.twist.angular.z;
}


void OdoTwistAngYaw::onQuit()
{
        disable();
}

void OdoTwistAngYaw::onPause()
{
        disable();
}

void OdoTwistAngYaw::onRun()
{
        enable( topic_name, (int)(size_queue()())   );
}

