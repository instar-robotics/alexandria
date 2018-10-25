/*
Copyright INSTAR Robotics

Author: Pierre Delarboulas

This software is governed by the CeCILL v2.1 license under French law and abiding by the rules of distribution of free software. 
You can use, modify and/ or redistribute the software under the terms of the CeCILL v2.1 license as circulated by CEA, CNRS and INRIA at the following URL "http://www.cecill.info".
As a counterpart to the access to the source code and  rights to copy, modify and redistribute granted by the license, 
users are provided only with a limited warranty and the software's author, the holder of the economic rights,  and the successive licensors have only limited liability.  
In this respect, the user's attention is drawn to the risks associated with loading, using, modifying and/or developing or reproducing the software by the user in light of its specific status of free software, 
that may mean  that it is complicated to manipulate, and that also therefore means that it is reserved for developers and experienced professionals having in-depth computer knowledge. 
Users are therefore encouraged to load and test the software's suitability as regards their requirements in conditions enabling the security of their systems and/or data to be ensured 
and, more generally, to use and operate it in the same conditions as regards security. 
The fact that you are presently reading this means that you have had knowledge of the CeCILL v2.1 license and that you accept its terms.
*/

#include "rosinput.h"
#include <tf/tf.h>
#include <algorithm>
#include <cmath>

REGISTER_FUNCTION(ScalarInput);
REGISTER_FUNCTION(MatrixInput);
REGISTER_FUNCTION(JoyAxesInput);
REGISTER_FUNCTION(JoyAxeInput);
REGISTER_FUNCTION(JoyButtonsInput);
REGISTER_FUNCTION(JoyButtonInput);
REGISTER_FUNCTION(OdoPosInput);
REGISTER_FUNCTION(OdoPosXInput);
REGISTER_FUNCTION(OdoPosYInput);
REGISTER_FUNCTION(OdoPosZInput);
REGISTER_FUNCTION(OdoEulerInput);
REGISTER_FUNCTION(OdoEulerRollInput);
REGISTER_FUNCTION(OdoEulerPitchInput);
REGISTER_FUNCTION(OdoEulerYawInput);
REGISTER_FUNCTION(OdoQuaterInput);
REGISTER_FUNCTION(OdoQuaterXInput);
REGISTER_FUNCTION(OdoQuaterYInput);
REGISTER_FUNCTION(OdoQuaterZInput);
REGISTER_FUNCTION(OdoQuaterWInput);
REGISTER_FUNCTION(OdoTwistLinInput);
REGISTER_FUNCTION(OdoTwistLinXInput);
REGISTER_FUNCTION(OdoTwistLinYInput);
REGISTER_FUNCTION(OdoTwistLinZInput);
REGISTER_FUNCTION(OdoTwistAngInput);
REGISTER_FUNCTION(OdoTwistAngRollInput);
REGISTER_FUNCTION(OdoTwistAngPitchInput);
REGISTER_FUNCTION(OdoTwistAngYawInput);
REGISTER_FUNCTION(LaserScanInput);

/*******************************************************************************************************/
/*****************                             ScalarInput                           *******************/
/*******************************************************************************************************/

void ScalarInput::compute()
{
	my_queue.callOne(ros::WallDuration(sleep()()));
}

void ScalarInput::setparameters()
{
 	Kernel::iBind(topic_name,"topic_name", getUuid());
 	Kernel::iBind(size_queue,"size_queue", getUuid());
 	Kernel::iBind(sleep,"sleep", getUuid());
}

void ScalarInput::callback(const std_msgs::Float64::ConstPtr &msg)
{
	output = msg->data;
}

void ScalarInput::onQuit()
{
        disable();
}

void ScalarInput::onPause()
{
        disable();
}

void ScalarInput::onRun()
{
        enable(topic_name, (int)(size_queue()()) );
}

/*******************************************************************************************************/
/*****************                             MatrixInput                           *******************/
/*******************************************************************************************************/

void MatrixInput::compute()
{
	my_queue.callOne(ros::WallDuration(sleep()()));
}

void MatrixInput::setparameters()
{
	Kernel::iBind(topic_name,"topic_name", getUuid());
 	Kernel::iBind(size_queue,"size_queue", getUuid());
 	Kernel::iBind(sleep,"sleep", getUuid());
}

void MatrixInput::callback( const std_msgs::Float64MultiArray::ConstPtr &msg)
{
	if( msg->layout.dim[0].size != output.rows() ||  msg->layout.dim[1].size !=  output.cols() ) 
	{
		throw std::invalid_argument("MatrixInput : Output dimension is not egal to the Float64MultiArray dimensions !");
	}

 	Map<const MatrixXd> mEnc (msg->data.data() , msg->layout.dim[0].size , msg->layout.dim[1].size );

       	output = mEnc;
}
void MatrixInput::onQuit()
{
        disable();
}

void MatrixInput::onPause()
{
        disable();
}

void MatrixInput::onRun()
{
        enable(topic_name, (int)(size_queue()()) );
}


/*******************************************************************************************************/
/*****************                           JoyAxesInput                            *******************/
/*******************************************************************************************************/

void JoyAxesInput::compute()
{
        my_queue.callOne(ros::WallDuration( sleep()()  ));
}

void JoyAxesInput::setparameters()
{
        Kernel::iBind(topic_name,"topic_name", getUuid());
 	Kernel::iBind(size_queue,"size_queue", getUuid());
 	Kernel::iBind(sleep,"sleep", getUuid());
}

void JoyAxesInput::callback( const sensor_msgs::Joy::ConstPtr &msg)
{
	if( msg->axes.size() != output.rows() * output.cols() ) 
	{
		throw std::invalid_argument("JoyAxesInput : Output dimension is not egal to the numbers of Joystick axes !");
	}

        auto mout = getMapVect(output);
	for(unsigned int i = 0; i < msg->axes.size() ; i++ )
	{
		mout[i] = msg->axes[i];	
	}
}

void JoyAxesInput::onQuit()
{
	disable();
}

void JoyAxesInput::onPause()
{
	disable();
}

void JoyAxesInput::onRun()
{
	enable(topic_name, (int)(size_queue()()) );
}
/*******************************************************************************************************/
/******************                           JoyAxeInput                            *******************/
/*******************************************************************************************************/

void JoyAxeInput::setparameters()
{
        Kernel::iBind(topic_name,"topic_name", getUuid());
        Kernel::iBind(size_queue,"size_queue", getUuid());
        Kernel::iBind(sleep,"sleep", getUuid());
        Kernel::iBind(axe,"axe", getUuid());
}

void JoyAxeInput::callback( const sensor_msgs::Joy::ConstPtr &msg)
{
        if( axe()() >  msg->axes.size())
        {
                throw std::invalid_argument("JoyAxeInput : axe ID is out of range !");
        }

	output = msg->axes[ (int)(axe()()) - 1];
}

void JoyAxeInput::compute()
{
       my_queue.callOne(ros::WallDuration( sleep()()  ));
}

void JoyAxeInput::onQuit()
{
	disable();
}

void JoyAxeInput::onPause()
{
	disable();
}

void JoyAxeInput::onRun()
{
	enable(topic_name, (int)(size_queue()()) );
}

/*******************************************************************************************************/
/*****************                         JoyButtonsInput                           *******************/
/*******************************************************************************************************/

void JoyButtonsInput::compute()
{
        my_queue.callOne(ros::WallDuration( sleep()()  ));
}

void JoyButtonsInput::setparameters()
{
        Kernel::iBind(topic_name,"topic_name", getUuid());
        Kernel::iBind(size_queue,"size_queue", getUuid());
        Kernel::iBind(sleep,"sleep", getUuid());
}

void JoyButtonsInput::callback( const sensor_msgs::Joy::ConstPtr &msg)
{
        if( msg->buttons.size() != output.rows() * output.cols() )
        {
                throw std::invalid_argument("JoyButtonsInput : Output dimension is not egal to the numbers of Joystick axes !");
        }

        auto mout = getMapVect(output);
        for(unsigned int i = 0; i < msg->buttons.size() ; i++ )
        {
                mout[i] = msg->buttons[i];
        }
}

void JoyButtonsInput::onQuit()
{
	disable();
}

void JoyButtonsInput::onPause()
{
	disable();
}

void JoyButtonsInput::onRun()
{
	enable(topic_name, (int)(size_queue()()) );
}

/*******************************************************************************************************/
/*****************                           JoyButtonInput                          *******************/
/*******************************************************************************************************/

void JoyButtonInput::compute()
{
        my_queue.callOne(ros::WallDuration( sleep()()  ));
}

void JoyButtonInput::setparameters()
{
        Kernel::iBind(topic_name,"topic_name", getUuid());
        Kernel::iBind(size_queue,"size_queue", getUuid());
        Kernel::iBind(sleep,"sleep", getUuid());
        Kernel::iBind(button,"button", getUuid());
}

void JoyButtonInput::callback( const sensor_msgs::Joy::ConstPtr &msg)
{
        if( button()() >  msg->buttons.size())
        {
                throw std::invalid_argument("JoyButtonInput : button ID is out of range !");
        }

        output = msg->buttons[ (int)(button()()) - 1];
}

void JoyButtonInput::onQuit()
{
	disable();
}

void JoyButtonInput::onPause()
{
	disable();
}

void JoyButtonInput::onRun()
{
	enable( topic_name, (int)(size_queue()())   );
}


/*******************************************************************************************************/
/*********************                         OdoPosInput                          ********************/
/*******************************************************************************************************/


void OdoPosInput::compute()
{
        my_queue.callOne(ros::WallDuration( sleep()() ));
}

void OdoPosInput::setparameters()
{
        Kernel::iBind(topic_name,"topic_name", getUuid());
        Kernel::iBind(size_queue,"size_queue", getUuid());
        Kernel::iBind(sleep,"sleep", getUuid());
}

void OdoPosInput::uprerun()
{
	if( output.rows() * output.cols() != 3 ) throw std::invalid_argument("OdoPosInput : Output dimension should be 3 !");
}

void OdoPosInput::callback(const nav_msgs::Odometry::ConstPtr &msg )
{
        auto mout = getMapVect(output);
        mout[0] = msg->pose.pose.position.x;
        mout[1] = msg->pose.pose.position.y;
        mout[2] = msg->pose.pose.position.z;
}

void OdoPosInput::onQuit()
{
	disable();
}

void OdoPosInput::onPause()
{
	disable();
}

void OdoPosInput::onRun()
{
	enable( topic_name, (int)(size_queue()())   );
}

/*******************************************************************************************************/
/*********************                        OdoPosXInput                          ********************/
/*******************************************************************************************************/


void OdoPosXInput::compute()
{
        my_queue.callOne(ros::WallDuration( sleep()() ));
}

void OdoPosXInput::setparameters()
{
        Kernel::iBind(topic_name,"topic_name", getUuid());
        Kernel::iBind(size_queue,"size_queue", getUuid());
        Kernel::iBind(sleep,"sleep", getUuid());
}

void OdoPosXInput::callback(const nav_msgs::Odometry::ConstPtr &msg )
{
	output = msg->pose.pose.position.x;
}

void OdoPosXInput::onQuit()
{
	disable();
}

void OdoPosXInput::onPause()
{
	disable();
}

void OdoPosXInput::onRun()
{
	enable( topic_name, (int)(size_queue()())   );
}
/*******************************************************************************************************/
/*********************                        OdoPosYInput                          ********************/
/*******************************************************************************************************/


void OdoPosYInput::compute()
{
        my_queue.callOne(ros::WallDuration( sleep()() ));
}

void OdoPosYInput::setparameters()
{
        Kernel::iBind(topic_name,"topic_name", getUuid());
        Kernel::iBind(size_queue,"size_queue", getUuid());
        Kernel::iBind(sleep,"sleep", getUuid());
}

void OdoPosYInput::callback(const nav_msgs::Odometry::ConstPtr &msg )
{
	output = msg->pose.pose.position.y;
}

void OdoPosYInput::onQuit()
{
	disable();
}

void OdoPosYInput::onPause()
{
	disable();
}

void OdoPosYInput::onRun()
{
	enable( topic_name, (int)(size_queue()())   );
}

/*******************************************************************************************************/
/*********************                        OdoPosZInput                          ********************/
/*******************************************************************************************************/

void OdoPosZInput::compute()
{
        my_queue.callOne(ros::WallDuration( sleep()() ));
}

void OdoPosZInput::setparameters()
{
        Kernel::iBind(topic_name,"topic_name", getUuid());
        Kernel::iBind(size_queue,"size_queue", getUuid());
        Kernel::iBind(sleep,"sleep", getUuid());
}

void OdoPosZInput::callback(const nav_msgs::Odometry::ConstPtr &msg )
{
	output = msg->pose.pose.position.z;
}

void OdoPosZInput::onQuit()
{
	disable();
}

void OdoPosZInput::onPause()
{
	disable();
}

void OdoPosZInput::onRun()
{
	enable( topic_name, (int)(size_queue()())   );
}

/*******************************************************************************************************/
/*********************                       OdoEulerInput                          ********************/
/*******************************************************************************************************/

void OdoEulerInput::compute()
{
        my_queue.callOne(ros::WallDuration( sleep()() ));
}

void OdoEulerInput::setparameters()
{
        Kernel::iBind(topic_name,"topic_name", getUuid());
        Kernel::iBind(size_queue,"size_queue", getUuid());
        Kernel::iBind(sleep,"sleep", getUuid());
}

void OdoEulerInput::uprerun()
{
	if( output.rows() * output.cols() != 3 ) throw std::invalid_argument("OdoPosInput : Output dimension should be 3 !");
}

void OdoEulerInput::callback(const nav_msgs::Odometry::ConstPtr &msg )
{
	tf::Quaternion q(msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z,msg->pose.pose.orientation.w);

	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);

	auto mout = getMapVect(output);
        mout[0] = roll;
        mout[1] = pitch;
        mout[2] = yaw;

}

void OdoEulerInput::onQuit()
{
	disable();
}

void OdoEulerInput::onPause()
{
	disable();
}

void OdoEulerInput::onRun()
{
	enable( topic_name, (int)(size_queue()())   );
}

/*******************************************************************************************************/
/*********************                    OdoEulerRollInput                         ********************/
/*******************************************************************************************************/

void OdoEulerRollInput::compute()
{
        my_queue.callOne(ros::WallDuration( sleep()() ));
}

void OdoEulerRollInput::setparameters()
{
        Kernel::iBind(topic_name,"topic_name", getUuid());
        Kernel::iBind(size_queue,"size_queue", getUuid());
        Kernel::iBind(sleep,"sleep", getUuid());
}

void OdoEulerRollInput::callback(const nav_msgs::Odometry::ConstPtr &msg )
{
	tf::Quaternion q(msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z,msg->pose.pose.orientation.w);

        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        output = roll;
}

void OdoEulerRollInput::onQuit()
{
	disable();
}

void OdoEulerRollInput::onPause()
{
	disable();
}

void OdoEulerRollInput::onRun()
{
	enable( topic_name, (int)(size_queue()())   );
}

/*******************************************************************************************************/
/*********************                   OdoEulerPitchInput                         ********************/
/*******************************************************************************************************/

void OdoEulerPitchInput::compute()
{
        my_queue.callOne(ros::WallDuration( sleep()() ));
}

void OdoEulerPitchInput::setparameters()
{
        Kernel::iBind(topic_name,"topic_name", getUuid());
        Kernel::iBind(size_queue,"size_queue", getUuid());
        Kernel::iBind(sleep,"sleep", getUuid());
}

void OdoEulerPitchInput::callback(const nav_msgs::Odometry::ConstPtr &msg )
{
	tf::Quaternion q(msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z,msg->pose.pose.orientation.w);

        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        output = pitch;
}

void OdoEulerPitchInput::onQuit()
{
	disable();
}

void OdoEulerPitchInput::onPause()
{
	disable();
}

void OdoEulerPitchInput::onRun()
{
	enable( topic_name, (int)(size_queue()())   );
}

/*******************************************************************************************************/
/*********************                     OdoEulerYawInput                         ********************/
/*******************************************************************************************************/

void OdoEulerYawInput::compute()
{
        my_queue.callOne(ros::WallDuration( sleep()() ));
}

void OdoEulerYawInput::setparameters()
{
        Kernel::iBind(topic_name,"topic_name", getUuid());
        Kernel::iBind(size_queue,"size_queue", getUuid());
        Kernel::iBind(sleep,"sleep", getUuid());
}

void OdoEulerYawInput::callback(const nav_msgs::Odometry::ConstPtr &msg )
{
	tf::Quaternion q(msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z,msg->pose.pose.orientation.w);

        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        output = yaw;
}

void OdoEulerYawInput::onQuit()
{
	disable();
}

void OdoEulerYawInput::onPause()
{
	disable();
}

void OdoEulerYawInput::onRun()
{
	enable( topic_name, (int)(size_queue()())   );
}

/*******************************************************************************************************/
/*********************                     OdoQuaterInput                         ********************/
/*******************************************************************************************************/

void OdoQuaterInput::compute()
{
        my_queue.callOne(ros::WallDuration( sleep()() ));
}

void OdoQuaterInput::setparameters()
{
        Kernel::iBind(topic_name,"topic_name", getUuid());
        Kernel::iBind(size_queue,"size_queue", getUuid());
        Kernel::iBind(sleep,"sleep", getUuid());
}

void OdoQuaterInput::uprerun()
{
	if( output.rows() * output.cols() != 4 ) throw std::invalid_argument("OdoQuaterInput : Output dimension should be 3 !");
}

void OdoQuaterInput::callback( const nav_msgs::Odometry::ConstPtr &msg )
{
        auto mout = getMapVect(output);
        mout[0] = msg->pose.pose.orientation.x;
        mout[1] = msg->pose.pose.orientation.y;
        mout[2] = msg->pose.pose.orientation.z;
        mout[3] = msg->pose.pose.orientation.w;
}

void OdoQuaterInput::onQuit()
{
	disable();
}

void OdoQuaterInput::onPause()
{
	disable();
}

void OdoQuaterInput::onRun()
{
	enable( topic_name, (int)(size_queue()())   );
}

/*******************************************************************************************************/
/*********************                     OdoQuaterXInput                         ********************/
/*******************************************************************************************************/

void OdoQuaterXInput::compute()
{
        my_queue.callOne(ros::WallDuration( sleep()() ));
}

void OdoQuaterXInput::setparameters()
{
        Kernel::iBind(topic_name,"topic_name", getUuid());
        Kernel::iBind(size_queue,"size_queue", getUuid());
        Kernel::iBind(sleep,"sleep", getUuid());
}

void OdoQuaterXInput::callback(const nav_msgs::Odometry::ConstPtr &msg )
{
        output = msg->pose.pose.orientation.x;
}

void OdoQuaterXInput::onQuit()
{
	disable();
}

void OdoQuaterXInput::onPause()
{
	disable();
}

void OdoQuaterXInput::onRun()
{
	enable( topic_name, (int)(size_queue()())   );
}
/*******************************************************************************************************/
/*********************                     OdoQuaterYInput                         ********************/
/*******************************************************************************************************/

void OdoQuaterYInput::compute()
{
        my_queue.callOne(ros::WallDuration( sleep()() ));
}

void OdoQuaterYInput::setparameters()
{
        Kernel::iBind(topic_name,"topic_name", getUuid());
        Kernel::iBind(size_queue,"size_queue", getUuid());
        Kernel::iBind(sleep,"sleep", getUuid());
}

void OdoQuaterYInput::callback( const nav_msgs::Odometry::ConstPtr &msg )
{
        output = msg->pose.pose.orientation.y;
}

void OdoQuaterYInput::onQuit()
{
	disable();
}

void OdoQuaterYInput::onPause()
{
	disable();
}

void OdoQuaterYInput::onRun()
{
	enable( topic_name, (int)(size_queue()())   );
}

/*******************************************************************************************************/
/*********************                     OdoQuaterZInput                         ********************/
/*******************************************************************************************************/

void OdoQuaterZInput::compute()
{
        my_queue.callOne(ros::WallDuration( sleep()() ));
}

void OdoQuaterZInput::setparameters()
{
        Kernel::iBind(topic_name,"topic_name", getUuid());
        Kernel::iBind(size_queue,"size_queue", getUuid());
        Kernel::iBind(sleep,"sleep", getUuid());
}

void OdoQuaterZInput::callback( const nav_msgs::Odometry::ConstPtr &msg )
{
        output = msg->pose.pose.orientation.z;
}

void OdoQuaterZInput::onQuit()
{
	disable();
}

void OdoQuaterZInput::onPause()
{
	disable();
}

void OdoQuaterZInput::onRun()
{
	enable( topic_name, (int)(size_queue()())   );
}
/*******************************************************************************************************/
/*********************                     OdoQuaterWInput                         ********************/
/*******************************************************************************************************/

void OdoQuaterWInput::compute()
{
        my_queue.callOne(ros::WallDuration( sleep()() ));
}

void OdoQuaterWInput::setparameters()
{
        Kernel::iBind(topic_name,"topic_name", getUuid());
        Kernel::iBind(size_queue,"size_queue", getUuid());
        Kernel::iBind(sleep,"sleep", getUuid());
}

void OdoQuaterWInput::callback(const nav_msgs::Odometry::ConstPtr &msg )
{
        output = msg->pose.pose.orientation.w;
}

void OdoQuaterWInput::onQuit()
{
	disable();
}

void OdoQuaterWInput::onPause()
{
	disable();
}

void OdoQuaterWInput::onRun()
{
	enable( topic_name, (int)(size_queue()())   );
}
/*******************************************************************************************************/
/*********************                     OdoTwistLinInput                         ********************/
/*******************************************************************************************************/

void OdoTwistLinInput::compute()
{
        my_queue.callOne(ros::WallDuration( sleep()() ));
}

void OdoTwistLinInput::setparameters()
{
        Kernel::iBind(topic_name,"topic_name", getUuid());
        Kernel::iBind(size_queue,"size_queue", getUuid());
        Kernel::iBind(sleep,"sleep", getUuid());
}

void OdoTwistLinInput::uprerun()
{
	if( output.rows() * output.cols() != 3 ) throw std::invalid_argument("OdoTwistLinInput : Output dimension should be 3 !");
}

void OdoTwistLinInput::callback(const nav_msgs::Odometry::ConstPtr &msg )
{
        auto mout = getMapVect(output);
        mout[0] = msg->twist.twist.linear.x;
        mout[1] = msg->twist.twist.linear.y;
        mout[2] = msg->twist.twist.linear.z;
}

void OdoTwistLinInput::onQuit()
{
	disable();
}

void OdoTwistLinInput::onPause()
{
	disable();
}

void OdoTwistLinInput::onRun()
{
	enable( topic_name, (int)(size_queue()())   );
}
/*******************************************************************************************************/
/*********************                    OdoTwistLinXInput                         ********************/
/*******************************************************************************************************/

void OdoTwistLinXInput::compute()
{
        my_queue.callOne(ros::WallDuration( sleep()() ));
}

void OdoTwistLinXInput::setparameters()
{
        Kernel::iBind(topic_name,"topic_name", getUuid());
        Kernel::iBind(size_queue,"size_queue", getUuid());
        Kernel::iBind(sleep,"sleep", getUuid());
}

void OdoTwistLinXInput::callback(const nav_msgs::Odometry::ConstPtr &msg )
{
        output = msg->twist.twist.linear.x;
}

void OdoTwistLinXInput::onQuit()
{
	disable();
}

void OdoTwistLinXInput::onPause()
{
	disable();
}

void OdoTwistLinXInput::onRun()
{
	enable( topic_name, (int)(size_queue()())   );
}
/*******************************************************************************************************/
/*********************                    OdoTwistLinYInput                         ********************/
/*******************************************************************************************************/

void OdoTwistLinYInput::compute()
{
        my_queue.callOne(ros::WallDuration( sleep()() ));
}

void OdoTwistLinYInput::setparameters()
{
        Kernel::iBind(topic_name,"topic_name", getUuid());
        Kernel::iBind(size_queue,"size_queue", getUuid());
        Kernel::iBind(sleep,"sleep", getUuid());
}

void OdoTwistLinYInput::callback(const nav_msgs::Odometry::ConstPtr &msg )
{
        output = msg->twist.twist.linear.y;
}


void OdoTwistLinYInput::onQuit()
{
	disable();
}

void OdoTwistLinYInput::onPause()
{
	disable();
}

void OdoTwistLinYInput::onRun()
{
	enable( topic_name, (int)(size_queue()())   );
}
/*******************************************************************************************************/
/*********************                    OdoTwistLinZInput                         ********************/
/*******************************************************************************************************/

void OdoTwistLinZInput::compute()
{
        my_queue.callOne(ros::WallDuration( sleep()() ));
}

void OdoTwistLinZInput::setparameters()
{
        Kernel::iBind(topic_name,"topic_name", getUuid());
        Kernel::iBind(size_queue,"size_queue", getUuid());
        Kernel::iBind(sleep,"sleep", getUuid());
}

void OdoTwistLinZInput::callback(const nav_msgs::Odometry::ConstPtr &msg )
{
        output = msg->twist.twist.linear.z;
}

void OdoTwistLinZInput::onQuit()
{
	disable();
}

void OdoTwistLinZInput::onPause()
{
	disable();
}

void OdoTwistLinZInput::onRun()
{
	enable( topic_name, (int)(size_queue()())   );
}

/*******************************************************************************************************/
/*********************                    OdoTwistAngInput                         ********************/
/*******************************************************************************************************/

void OdoTwistAngInput::compute()
{
        my_queue.callOne(ros::WallDuration( sleep()() ));
}

void OdoTwistAngInput::setparameters()
{
        Kernel::iBind(topic_name,"topic_name", getUuid());
        Kernel::iBind(size_queue,"size_queue", getUuid());
        Kernel::iBind(sleep,"sleep", getUuid());
}

void OdoTwistAngInput::uprerun()
{
	if( output.rows() * output.cols() != 3 ) throw std::invalid_argument("OdoTwistAngInput : Output dimension should be 3 !");
}

void OdoTwistAngInput::callback(const nav_msgs::Odometry::ConstPtr &msg )
{
        auto mout = getMapVect(output);
        mout[0] = msg->twist.twist.angular.x;
        mout[1] = msg->twist.twist.angular.y;
        mout[2] = msg->twist.twist.angular.z;
}

void OdoTwistAngInput::onQuit()
{
	disable();
}

void OdoTwistAngInput::onPause()
{
	disable();
}

void OdoTwistAngInput::onRun()
{
	enable( topic_name, (int)(size_queue()())   );
}

/*******************************************************************************************************/
/*********************                  OdoTwistAngRollInput                        ********************/
/*******************************************************************************************************/

void OdoTwistAngRollInput::compute()
{
        my_queue.callOne(ros::WallDuration( sleep()() ));
}

void OdoTwistAngRollInput::setparameters()
{
        Kernel::iBind(topic_name,"topic_name", getUuid());
        Kernel::iBind(size_queue,"size_queue", getUuid());
        Kernel::iBind(sleep,"sleep", getUuid());
}

void OdoTwistAngRollInput::callback(const nav_msgs::Odometry::ConstPtr &msg )
{
        output = msg->twist.twist.angular.x;
}

void OdoTwistAngRollInput::onQuit()
{
	disable();
}

void OdoTwistAngRollInput::onPause()
{
	disable();
}

void OdoTwistAngRollInput::onRun()
{
	enable( topic_name, (int)(size_queue()())   );
}
/*******************************************************************************************************/
/*********************                  OdoTwistAngPitchInput                       ********************/
/*******************************************************************************************************/

void OdoTwistAngPitchInput::compute()
{
        my_queue.callOne(ros::WallDuration( sleep()() ));
}

void OdoTwistAngPitchInput::setparameters()
{
        Kernel::iBind(topic_name,"topic_name", getUuid());
        Kernel::iBind(size_queue,"size_queue", getUuid());
        Kernel::iBind(sleep,"sleep", getUuid());
}

void OdoTwistAngPitchInput::callback(const nav_msgs::Odometry::ConstPtr &msg )
{
        output = msg->twist.twist.angular.y;
}

void OdoTwistAngPitchInput::onQuit()
{
	disable();
}

void OdoTwistAngPitchInput::onPause()
{
	disable();
}

void OdoTwistAngPitchInput::onRun()
{
	enable( topic_name, (int)(size_queue()())   );
}

/*******************************************************************************************************/
/*********************                    OdoTwistAngYawInput                       ********************/
/*******************************************************************************************************/

void OdoTwistAngYawInput::compute()
{
        my_queue.callOne(ros::WallDuration( sleep()() ));
}

void OdoTwistAngYawInput::setparameters()
{
        Kernel::iBind(topic_name,"topic_name", getUuid());
        Kernel::iBind(size_queue,"size_queue", getUuid());
        Kernel::iBind(sleep,"sleep", getUuid());
}

void OdoTwistAngYawInput::callback(const nav_msgs::Odometry::ConstPtr &msg )
{
        output = msg->twist.twist.angular.z;
}


void OdoTwistAngYawInput::onQuit()
{
	disable();
}

void OdoTwistAngYawInput::onPause()
{
	disable();
}

void OdoTwistAngYawInput::onRun()
{
	enable( topic_name, (int)(size_queue()())   );
}


/*******************************************************************************************************/
/*********************                       LaserScanInput                         ********************/
/*******************************************************************************************************/

void LaserScanInput::compute()
{
        my_queue.callOne(ros::WallDuration( sleep()() ));
}

void LaserScanInput::setparameters()
{
	if( output.rows() > 1 )  throw std::invalid_argument("LaserScanInput : Output must be a Vector [ROW = 1 and Cols = N]");

	moy.assign(output.cols(), 0);

        Kernel::iBind(topic_name,"topic_name", getUuid());
        Kernel::iBind(size_queue,"size_queue", getUuid());
        Kernel::iBind(sleep,"sleep", getUuid());
        Kernel::iBind(range_max,"range_max", getUuid());
}

void LaserScanInput::callback(const sensor_msgs::LaserScan::ConstPtr &msg )
{
	double RM = std::min(range_max()(),  (double)(msg->range_max) ); 
	double offset =  M_PI - fabs(msg->angle_min);

	for( unsigned int i = 0 ; i < msg->ranges.size() ; i++)
	{
		unsigned int j = ( i * (msg->angle_max - msg->angle_min) /  msg->ranges.size() + offset ) * ( output.cols() / (2* M_PI)) ;

		double value = 1 - (msg->ranges[i] - msg->range_min) / (RM - msg->range_min) ;
		if( value < 0 ) value = 0;

		if( moy[j] == 0 ) output(0,j) = value;
		else output(0,j) += value;

		moy[j]++;
	}

	for(unsigned int i = 0 ; i < output.cols() ; i++)
	{

		if( moy[i] > 0 ) output(0,i) = output(0,i) / moy[i];
		else output(0,i) = 0;

		moy[i] = 0;
	}
}

void LaserScanInput::onQuit()
{
        disable();
}

void LaserScanInput::onPause()
{
        disable();
}

void LaserScanInput::onRun()
{
        enable( topic_name, (int)(size_queue()())   );
}

