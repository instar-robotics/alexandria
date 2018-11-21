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
REGISTER_FUNCTION(JoyAxes);
REGISTER_FUNCTION(JoyAxe);
REGISTER_FUNCTION(JoyButtons);
REGISTER_FUNCTION(JoyButton);
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
REGISTER_FUNCTION(Lidar1D);
REGISTER_FUNCTION(Lidar2D);

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
		std::cout << "DIM 0 : " << msg->layout.dim[0].size << " DIM 1 : " << msg->layout.dim[1].size << std::endl; 

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
/*****************                           JoyAxes                            *******************/
/*******************************************************************************************************/

void JoyAxes::compute()
{
        my_queue.callOne(ros::WallDuration( sleep()()  ));
}

void JoyAxes::setparameters()
{
        Kernel::iBind(topic_name,"topic_name", getUuid());
 	Kernel::iBind(size_queue,"size_queue", getUuid());
 	Kernel::iBind(sleep,"sleep", getUuid());
}

void JoyAxes::callback( const sensor_msgs::Joy::ConstPtr &msg)
{
	if( msg->axes.size() != output.rows() * output.cols() ) 
	{
		throw std::invalid_argument("JoyAxes : Output dimension is not egal to the numbers of Joystick axes !");
	}

        auto mout = getMapVect(output);
	for(unsigned int i = 0; i < msg->axes.size() ; i++ )
	{
		mout[i] = msg->axes[i];	
	}
}

void JoyAxes::onQuit()
{
	disable();
}

void JoyAxes::onPause()
{
	disable();
}

void JoyAxes::onRun()
{
	enable(topic_name, (int)(size_queue()()) );
}
/*******************************************************************************************************/
/******************                           JoyAxe                            *******************/
/*******************************************************************************************************/

void JoyAxe::setparameters()
{
        Kernel::iBind(topic_name,"topic_name", getUuid());
        Kernel::iBind(size_queue,"size_queue", getUuid());
        Kernel::iBind(sleep,"sleep", getUuid());
        Kernel::iBind(axe,"axe", getUuid());
}

void JoyAxe::callback( const sensor_msgs::Joy::ConstPtr &msg)
{
        if( axe()() >  msg->axes.size())
        {
                throw std::invalid_argument("JoyAxe : axe ID is out of range !");
        }

	output = msg->axes[ (int)(axe()()) - 1];
}

void JoyAxe::compute()
{
       my_queue.callOne(ros::WallDuration( sleep()()  ));
}

void JoyAxe::onQuit()
{
	disable();
}

void JoyAxe::onPause()
{
	disable();
}

void JoyAxe::onRun()
{
	enable(topic_name, (int)(size_queue()()) );
}

/*******************************************************************************************************/
/*****************                         JoyButtons                           *******************/
/*******************************************************************************************************/

void JoyButtons::compute()
{
        my_queue.callOne(ros::WallDuration( sleep()()  ));
}

void JoyButtons::setparameters()
{
        Kernel::iBind(topic_name,"topic_name", getUuid());
        Kernel::iBind(size_queue,"size_queue", getUuid());
        Kernel::iBind(sleep,"sleep", getUuid());
}

void JoyButtons::callback( const sensor_msgs::Joy::ConstPtr &msg)
{
        if( msg->buttons.size() != output.rows() * output.cols() )
        {
                throw std::invalid_argument("JoyButtons : Output dimension is not egal to the numbers of Joystick axes !");
        }

        auto mout = getMapVect(output);
        for(unsigned int i = 0; i < msg->buttons.size() ; i++ )
        {
                mout[i] = msg->buttons[i];
        }
}

void JoyButtons::onQuit()
{
	disable();
}

void JoyButtons::onPause()
{
	disable();
}

void JoyButtons::onRun()
{
	enable(topic_name, (int)(size_queue()()) );
}

/*******************************************************************************************************/
/*****************                           JoyButton                          *******************/
/*******************************************************************************************************/

void JoyButton::compute()
{
        my_queue.callOne(ros::WallDuration( sleep()()  ));
}

void JoyButton::setparameters()
{
        Kernel::iBind(topic_name,"topic_name", getUuid());
        Kernel::iBind(size_queue,"size_queue", getUuid());
        Kernel::iBind(sleep,"sleep", getUuid());
        Kernel::iBind(button,"button", getUuid());
}

void JoyButton::callback( const sensor_msgs::Joy::ConstPtr &msg)
{
        if( button()() >  msg->buttons.size())
        {
                throw std::invalid_argument("JoyButton : button ID is out of range !");
        }

        output = msg->buttons[ (int)(button()()) - 1];
}

void JoyButton::onQuit()
{
	disable();
}

void JoyButton::onPause()
{
	disable();
}

void JoyButton::onRun()
{
	enable( topic_name, (int)(size_queue()())   );
}


/*******************************************************************************************************/
/*********************                         OdoPos                          ********************/
/*******************************************************************************************************/


void OdoPos::compute()
{
        my_queue.callOne(ros::WallDuration( sleep()() ));
}

void OdoPos::setparameters()
{
        Kernel::iBind(topic_name,"topic_name", getUuid());
        Kernel::iBind(size_queue,"size_queue", getUuid());
        Kernel::iBind(sleep,"sleep", getUuid());
}

void OdoPos::uprerun()
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
/*********************                        OdoPosX                          ********************/
/*******************************************************************************************************/


void OdoPosX::compute()
{
        my_queue.callOne(ros::WallDuration( sleep()() ));
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
/*********************                        OdoPosY                          ********************/
/*******************************************************************************************************/


void OdoPosY::compute()
{
        my_queue.callOne(ros::WallDuration( sleep()() ));
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
/*********************                        OdoPosZ                          ********************/
/*******************************************************************************************************/

void OdoPosZ::compute()
{
        my_queue.callOne(ros::WallDuration( sleep()() ));
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
/*********************                       OdoEuler                          ********************/
/*******************************************************************************************************/

void OdoEuler::compute()
{
        my_queue.callOne(ros::WallDuration( sleep()() ));
}

void OdoEuler::setparameters()
{
        Kernel::iBind(topic_name,"topic_name", getUuid());
        Kernel::iBind(size_queue,"size_queue", getUuid());
        Kernel::iBind(sleep,"sleep", getUuid());
}

void OdoEuler::uprerun()
{
	if( output.rows() * output.cols() != 3 ) throw std::invalid_argument("OdoEuler : Output dimension should be 3 !");
}

void OdoEuler::callback(const nav_msgs::Odometry::ConstPtr &msg )
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
/*********************                    OdoEulerRoll                         ********************/
/*******************************************************************************************************/

void OdoEulerRoll::compute()
{
        my_queue.callOne(ros::WallDuration( sleep()() ));
}

void OdoEulerRoll::setparameters()
{
        Kernel::iBind(topic_name,"topic_name", getUuid());
        Kernel::iBind(size_queue,"size_queue", getUuid());
        Kernel::iBind(sleep,"sleep", getUuid());
}

void OdoEulerRoll::callback(const nav_msgs::Odometry::ConstPtr &msg )
{
	tf::Quaternion q(msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z,msg->pose.pose.orientation.w);

        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
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
/*********************                   OdoEulerPitch                         ********************/
/*******************************************************************************************************/

void OdoEulerPitch::compute()
{
        my_queue.callOne(ros::WallDuration( sleep()() ));
}

void OdoEulerPitch::setparameters()
{
        Kernel::iBind(topic_name,"topic_name", getUuid());
        Kernel::iBind(size_queue,"size_queue", getUuid());
        Kernel::iBind(sleep,"sleep", getUuid());
}

void OdoEulerPitch::callback(const nav_msgs::Odometry::ConstPtr &msg )
{
	tf::Quaternion q(msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z,msg->pose.pose.orientation.w);

        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
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
/*********************                     OdoEulerYaw                         ********************/
/*******************************************************************************************************/

void OdoEulerYaw::compute()
{
        my_queue.callOne(ros::WallDuration( sleep()() ));
}

void OdoEulerYaw::setparameters()
{
        Kernel::iBind(topic_name,"topic_name", getUuid());
        Kernel::iBind(size_queue,"size_queue", getUuid());
        Kernel::iBind(sleep,"sleep", getUuid());
}

void OdoEulerYaw::callback(const nav_msgs::Odometry::ConstPtr &msg )
{
	tf::Quaternion q(msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z,msg->pose.pose.orientation.w);

        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
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
/*********************                     OdoQuater                         ********************/
/*******************************************************************************************************/

void OdoQuater::compute()
{
        my_queue.callOne(ros::WallDuration( sleep()() ));
}

void OdoQuater::setparameters()
{
        Kernel::iBind(topic_name,"topic_name", getUuid());
        Kernel::iBind(size_queue,"size_queue", getUuid());
        Kernel::iBind(sleep,"sleep", getUuid());
}

void OdoQuater::uprerun()
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
/*********************                     OdoQuaterX                         ********************/
/*******************************************************************************************************/

void OdoQuaterX::compute()
{
        my_queue.callOne(ros::WallDuration( sleep()() ));
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
/*********************                     OdoQuaterY                         ********************/
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
/*********************                     OdoQuaterZ                         ********************/
/*******************************************************************************************************/

void OdoQuaterZ::compute()
{
        my_queue.callOne(ros::WallDuration( sleep()() ));
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
/*********************                     OdoQuaterW                         ********************/
/*******************************************************************************************************/

void OdoQuaterW::compute()
{
        my_queue.callOne(ros::WallDuration( sleep()() ));
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
/*********************                     OdoTwistLin                         ********************/
/*******************************************************************************************************/

void OdoTwistLin::compute()
{
        my_queue.callOne(ros::WallDuration( sleep()() ));
}

void OdoTwistLin::setparameters()
{
        Kernel::iBind(topic_name,"topic_name", getUuid());
        Kernel::iBind(size_queue,"size_queue", getUuid());
        Kernel::iBind(sleep,"sleep", getUuid());
}

void OdoTwistLin::uprerun()
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
/*********************                    OdoTwistLinX                         ********************/
/*******************************************************************************************************/

void OdoTwistLinX::compute()
{
        my_queue.callOne(ros::WallDuration( sleep()() ));
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
/*********************                    OdoTwistLinY                         ********************/
/*******************************************************************************************************/

void OdoTwistLinY::compute()
{
        my_queue.callOne(ros::WallDuration( sleep()() ));
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
/*********************                    OdoTwistLinZ                         ********************/
/*******************************************************************************************************/

void OdoTwistLinZ::compute()
{
        my_queue.callOne(ros::WallDuration( sleep()() ));
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
/*********************                    OdoTwistAng                         ********************/
/*******************************************************************************************************/

void OdoTwistAng::compute()
{
        my_queue.callOne(ros::WallDuration( sleep()() ));
}

void OdoTwistAng::setparameters()
{
        Kernel::iBind(topic_name,"topic_name", getUuid());
        Kernel::iBind(size_queue,"size_queue", getUuid());
        Kernel::iBind(sleep,"sleep", getUuid());
}

void OdoTwistAng::uprerun()
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
/*********************                  OdoTwistAngRoll                        ********************/
/*******************************************************************************************************/

void OdoTwistAngRoll::compute()
{
        my_queue.callOne(ros::WallDuration( sleep()() ));
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
/*********************                  OdoTwistAngPitch                       ********************/
/*******************************************************************************************************/

void OdoTwistAngPitch::compute()
{
        my_queue.callOne(ros::WallDuration( sleep()() ));
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
/*********************                    OdoTwistAngYaw                       ********************/
/*******************************************************************************************************/

void OdoTwistAngYaw::compute()
{
        my_queue.callOne(ros::WallDuration( sleep()() ));
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


/*******************************************************************************************************/
/************************                       Lidar1D                        *************************/
/*******************************************************************************************************/

void Lidar1D::compute()
{
        my_queue.callOne(ros::WallDuration( sleep()() ));
}

void Lidar1D::setparameters()
{
	if( output.rows() > 1 )  throw std::invalid_argument("Lidar1D : Output must be a Vector [ROW = 1 and Cols = N]");

	moy.assign(output.cols(), 0);

        Kernel::iBind(topic_name,"topic_name", getUuid());
        Kernel::iBind(size_queue,"size_queue", getUuid());
        Kernel::iBind(sleep,"sleep", getUuid());
        Kernel::iBind(range_max,"range_max", getUuid());
}

void Lidar1D::callback(const sensor_msgs::LaserScan::ConstPtr &msg )
{
        double RM = std::min(range_max()(),  (double)(msg->range_max) );
        double offset =  M_PI - fabs(msg->angle_min);

        for( unsigned int i = 0 ; i <  msg->ranges.size() ; i++)
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

void Lidar1D::onQuit()
{
        disable();
}

void Lidar1D::onPause()
{
        disable();
}

void Lidar1D::onRun()
{
        enable( topic_name, (int)(size_queue()())   );
}


/*******************************************************************************************************/
/************************                       Lidar2D                        *************************/
/*******************************************************************************************************/

void Lidar2D::compute()
{
        my_queue.callOne(ros::WallDuration( sleep()() ));
}

void Lidar2D::setparameters()
{
        Kernel::iBind(topic_name,"topic_name", getUuid());
        Kernel::iBind(size_queue,"size_queue", getUuid());
        Kernel::iBind(sleep,"sleep", getUuid());
        Kernel::iBind(range_max,"range_max", getUuid());

	moy = MatrixXi::Constant(output.rows(), output.cols(),0);
}

//void Lidar2D::callback(const sensor_msgs::PointCloud2::ConstPtr &msg )
void Lidar2D::callback(const PointCloud::ConstPtr &msg )
{
	output = MatrixXd::Constant(output.rows(),output.cols(),0);

	for( unsigned int i = 0 ; i <  msg->points.size() ; i++ )
	{
		

		 double theta = (2 * atan( msg->points[i].y / ( msg->points[i].x + sqrt( msg->points[i].x * msg->points[i].x + msg->points[i].y * msg->points[i].y   ))) + M_PI ) *  output.cols() / (2*M_PI);

		 double value = sqrt( msg->points[i].x * msg->points[i].x + msg->points[i].y * msg->points[i].y + msg->points[i].z * msg->points[i].z  );

		 std::cout <<  msg->points[i].z+10  << " " << theta << " "<< value << std::endl;
		 if( ! std::isnan(theta) )  output( msg->points[i].z+10 , theta) = value;
	}

/*
	std::cout << msg->points.size() << std::endl;

	std::cout << msg->width << " " << msg->height << std::endl;

	auto const m = msg->getMatrixXfMap();
	std::cout << m.rows() << " . " << m.cols() << std::endl;

	for( unsigned int i = 0 ; i < m.cols() ; i++)
		for(unsigned int j = 0; j < m.rows(); j++)
	{
		output( j * output.rows() / m.rows() , i * output.cols() / m.cols() ) = m(j,i);	
	}
*/


//	std::cout << m << std::endl;


	/*
	for( unsigned int i = 0 ; i <  msg->points.size() ; i++ )
	{
		 std::cout << msg->points[i].x << "  " << msg->points[i].y << " " <<  msg->points[i].z << std::endl;
	}
	*/
	/*
	std::cout << "Width : " << msg->width << std::endl;
	std::cout << "Height : " << msg->height << std::endl;
	std::cout << "Point Step : " << msg->point_step << std::endl;
	std::cout << "Row Step : " << msg->row_step << std::endl;


	std::cout << "Point Fields Size : " << msg->fields.size() << std::endl;

	for(unsigned int i =0 ; i < msg->fields.size() ; i++)
	{
		std::cout << "\tPF Name : " << msg->fields[i].name << std::endl;
		std::cout << "\tPF offset : " << msg->fields[i].offset << std::endl;
		std::cout << "\tPF datatype : " << (int)msg->fields[i].datatype << std::endl;
		std::cout << "\tPF count : " << msg->fields[i].count << std::endl;

	}*/
}

void Lidar2D::onQuit()
{
        disable();
}

void Lidar2D::onPause()
{
        disable();
}

void Lidar2D::onRun()
{
        enable( topic_name, (int)(size_queue()())   );
}

