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

#include "nav_pub.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseStamped.h>

REGISTER_FUNCTION(PathPub);

/*******************************************************************************************************/
/********************                            PathPub                              ******************/
/*******************************************************************************************************/

void PathPub::compute()
{
        auto mout = getMapVect(output);

        MATRIX tmpO = position()();

        auto mtmp = getMapVect(tmpO);
        mout[0] = mtmp[0];
        mout[1] = mtmp[1];
        mout[2] = mtmp[2];

        msg.poses[0].pose.position.x = mout[0];
        msg.poses[0].pose.position.y = mout[1];
        msg.poses[0].pose.position.z = mout[2];

        tmpO = orientation()();
        mtmp = getMapVect(tmpO);
        mout[3] = mtmp[0];
        mout[4] = mtmp[1];
        mout[5] = mtmp[2];

        tf2::Quaternion quat;
        quat.setRPY( mout[3], mout[4], mout[5] );

        msg.poses[0].pose.orientation = tf2::toMsg(quat);
        pub.publish(msg);
}

void PathPub::setparameters()
{
        if( output.size() != 6 ) throw std::invalid_argument("PathPub : Output must be a 6D vector");

        FMatrixPub<nav_msgs::Path>::setparameters();
        position.setCheckSize(false);
        orientation.setCheckSize(false);
        Kernel::iBind(frame_id,"frame_id", getUuid());
        Kernel::iBind(position,"position", getUuid());
        Kernel::iBind(orientation,"orientation", getUuid());
}

void PathPub::prerun()
{
        if( position().iSize() != 3 || !position().isVect()) throw std::invalid_argument("PathPub : position must be a 3D Vector.");
        if( orientation().iSize() != 3 || !orientation().isVect()) throw std::invalid_argument("PathPub : orientation must be a 3D Vector.");

        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = frame_id;
        msg.header.frame_id = frame_id;
	msg.poses.push_back(pose);

        FMatrixPub<nav_msgs::Path>::prerun();
}
