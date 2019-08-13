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

#include "tf.h"
#include <geometry_msgs/TransformStamped.h>

REGISTER_FUNCTION(TFListener);

/***************************************************************************************************/
/****************************************   TFListener   *******************************************/
/***************************************************************************************************/

void TFListener::setparameters()
{
	if(output.size() != 7) throw std::invalid_argument("TFListener : Output must be a 7D Vector.");

	Kernel::iBind(target_frame,"target_frame", getUuid());
	Kernel::iBind(source_frame,"target_frame", getUuid());
	Kernel::iBind(time,"time", getUuid());
	Kernel::iBind(timeout,"timeout", getUuid());
}

void TFListener::compute()
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
