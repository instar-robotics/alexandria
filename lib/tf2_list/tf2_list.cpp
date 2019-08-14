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

#include "tf2_list.h"
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

REGISTER_FUNCTION(TF2Listener);
REGISTER_FUNCTION(TF2ListenerTrans);
REGISTER_FUNCTION(TF2ListenerEuler);
REGISTER_FUNCTION(TF2ListenerEulerYaw);
REGISTER_FUNCTION(TF2ListenerQuater);

/***************************************************************************************************/
/****************************************   TF2Listener   *******************************************/
/***************************************************************************************************/

void TF2Listener::setparameters()
{
	if(output.size() != 7) throw std::invalid_argument("TF2Listener : Output must be a 7D Vector.");

	Kernel::iBind(target_frame,"target_frame", getUuid());
	Kernel::iBind(source_frame,"source_frame", getUuid());
	Kernel::iBind(time,"time", getUuid());
	Kernel::iBind(timeout,"timeout", getUuid());
}

void TF2Listener::compute()
{
	tf2_ros::TransformListener tfListener(tfBuffer);	
	auto mout = getMapVect(output);

	geometry_msgs::TransformStamped transformStamped;

	try
	{
		transformStamped = tfBuffer.lookupTransform( target_frame , source_frame, ros::Time(time()()), ros::Duration(timeout()()) );


		mout[0] = transformStamped.transform.translation.x;
		mout[1] = transformStamped.transform.translation.y;
		mout[2] = transformStamped.transform.translation.z;
		mout[3] = transformStamped.transform.rotation.x;
		mout[4] = transformStamped.transform.rotation.y;
		mout[5] = transformStamped.transform.rotation.z;
		mout[6] = transformStamped.transform.rotation.w;
	}
	catch (tf2::TransformException &ex) 
	{
		output = MATRIX::Constant(output.rows(), output.cols(), 0);
	}
}


/***************************************************************************************************/
/**************************************   TFListenerTans   ******************************************/
/***************************************************************************************************/

void TF2ListenerTrans::setparameters()
{
        if(output.size() != 3) throw std::invalid_argument("TF2ListenerTrans : Output must be a 3D Vector.");

        Kernel::iBind(target_frame,"target_frame", getUuid());
        Kernel::iBind(source_frame,"source_frame", getUuid());
        Kernel::iBind(time,"time", getUuid());
        Kernel::iBind(timeout,"timeout", getUuid());
}

void TF2ListenerTrans::compute()
{
        tf2_ros::TransformListener tfListener(tfBuffer);        
        auto mout = getMapVect(output);

        geometry_msgs::TransformStamped transformStamped;

        try
        {
                transformStamped = tfBuffer.lookupTransform( target_frame , source_frame, ros::Time(time()()), ros::Duration(timeout()()) );


                mout[0] = transformStamped.transform.translation.x;
                mout[1] = transformStamped.transform.translation.y;
                mout[2] = transformStamped.transform.translation.z;
        }
        catch (tf2::TransformException &ex)
        {
                output = MATRIX::Constant(output.rows(), output.cols(), 0);
        }
}


/***************************************************************************************************/
/*************************************   TFListenerEuler   *****************************************/
/***************************************************************************************************/

void TF2ListenerEuler::setparameters()
{
        if(output.size() != 3) throw std::invalid_argument("TF2ListenerEuler : Output must be a 3D Vector.");

        Kernel::iBind(target_frame,"target_frame", getUuid());
        Kernel::iBind(source_frame,"source_frame", getUuid());
        Kernel::iBind(time,"time", getUuid());
        Kernel::iBind(timeout,"timeout", getUuid());
}

void TF2ListenerEuler::compute()
{
        tf2_ros::TransformListener tfListener(tfBuffer);
        auto mout = getMapVect(output);

        geometry_msgs::TransformStamped transformStamped;

        try
        {
                transformStamped = tfBuffer.lookupTransform( target_frame , source_frame, ros::Time(time()()), ros::Duration(timeout()()));

                tf2::Quaternion q(transformStamped.transform.rotation.x , transformStamped.transform.rotation.y , transformStamped.transform.rotation.z , transformStamped.transform.rotation.w);
		tf2::Matrix3x3 m(q);
                m.getRPY( mout[0] , mout[1] , mout[2]);
        }
        catch (tf2::TransformException &ex)
        {
                output = MATRIX::Constant(output.rows(), output.cols(), 0);
        }
}


void TF2ListenerEulerYaw::setparameters()
{
        Kernel::iBind(target_frame,"target_frame", getUuid());
        Kernel::iBind(source_frame,"source_frame", getUuid());
        Kernel::iBind(time,"time", getUuid());
        Kernel::iBind(timeout,"timeout", getUuid());
}

void TF2ListenerEulerYaw::compute()
{
        tf2_ros::TransformListener tfListener(tfBuffer);

        geometry_msgs::TransformStamped transformStamped;

        try
        {
                transformStamped = tfBuffer.lookupTransform( target_frame , source_frame, ros::Time(time()()), ros::Duration(timeout()()));

                tf2::Quaternion q(transformStamped.transform.rotation.x , transformStamped.transform.rotation.y , transformStamped.transform.rotation.z , transformStamped.transform.rotation.w);
                tf2::Matrix3x3 m(q);
		SCALAR roll, pitch, yaw;
                m.getRPY(roll, pitch, yaw);
                output = yaw;
        }
        catch (tf2::TransformException &ex)
        {
                output = 0;
        }
}

/***************************************************************************************************/
/*************************************   TFListenerQuater   ****************************************/
/***************************************************************************************************/

void TF2ListenerQuater::setparameters()
{
        if(output.size() != 4) throw std::invalid_argument("TF2ListenerQuater : Output must be a 4D Vector.");

        Kernel::iBind(target_frame,"target_frame", getUuid());
        Kernel::iBind(source_frame,"source_frame", getUuid());
        Kernel::iBind(time,"time", getUuid());
        Kernel::iBind(timeout,"timeout", getUuid());
}

void TF2ListenerQuater::compute()
{
        tf2_ros::TransformListener tfListener(tfBuffer);        
        auto mout = getMapVect(output);

        geometry_msgs::TransformStamped transformStamped;

        try
        {
                transformStamped = tfBuffer.lookupTransform( target_frame , source_frame, ros::Time(time()()), ros::Duration(timeout()()) );

                mout[0] = transformStamped.transform.rotation.x;
                mout[1] = transformStamped.transform.rotation.y;
                mout[2] = transformStamped.transform.rotation.z;
                mout[3] = transformStamped.transform.rotation.w;
        }
        catch (tf2::TransformException &ex)
        {
                output = MATRIX::Constant(output.rows(), output.cols(), 0);
        }
}

