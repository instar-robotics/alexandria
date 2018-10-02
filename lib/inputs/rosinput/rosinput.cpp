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

void ScalarInput::uprerun()
{
	subscribe(topic_name, (int)(size_queue()()) );
}

void ScalarInput::callback(const std_msgs::Float64::ConstPtr &msg)
{
	output = msg->data;
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

void MatrixInput::uprerun()
{
	subscribe(topic_name, (int)(size_queue()()) );
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

void JoyAxesInput::uprerun()
{
	subscribe(topic_name, (int)(size_queue()()) );
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

/*******************************************************************************************************/
/******************                           JoyAxeInput                            *******************/
/*******************************************************************************************************/

void JoyAxeInput::compute()
{
        my_queue.callOne(ros::WallDuration( sleep()()  ));
}

void JoyAxeInput::setparameters()
{
        Kernel::iBind(topic_name,"topic_name", getUuid());
        Kernel::iBind(size_queue,"size_queue", getUuid());
        Kernel::iBind(sleep,"sleep", getUuid());
        Kernel::iBind(axe,"axe", getUuid());
}

void JoyAxeInput::uprerun()
{
        subscribe(topic_name, (int)(size_queue()()) );
}

void JoyAxeInput::callback( const sensor_msgs::Joy::ConstPtr &msg)
{
        if( axe()() >  msg->axes.size())
        {
                throw std::invalid_argument("JoyAxeInput : axe ID is out of range !");
        }

	output = msg->axes[ (int)(axe()()) - 1];
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

void JoyButtonsInput::uprerun()
{
        subscribe(topic_name, (int)(size_queue()()) );
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

void JoyButtonInput::uprerun()
{
        subscribe(topic_name, (int)(size_queue()()) );
}

void JoyButtonInput::callback( const sensor_msgs::Joy::ConstPtr &msg)
{
        if( button()() >  msg->buttons.size())
        {
                throw std::invalid_argument("JoyButtonInput : button ID is out of range !");
        }

        output = msg->buttons[ (int)(button()()) - 1];
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
        subscribe(topic_name, (int)(size_queue()()) );
}

void OdoPosInput::callback(const nav_msgs::Odometry::ConstPtr &msg )
{
        auto mout = getMapVect(output);
        mout[0] = msg->pose.pose.position.x;
        mout[1] = msg->pose.pose.position.y;
        mout[2] = msg->pose.pose.position.z;
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

void OdoPosXInput::uprerun()
{
        subscribe(topic_name, (int)(size_queue()()) );
}

void OdoPosXInput::callback(const nav_msgs::Odometry::ConstPtr &msg )
{
	output = msg->pose.pose.position.x;
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

void OdoPosYInput::uprerun()
{
        subscribe(topic_name, (int)(size_queue()()) );
}

void OdoPosYInput::callback(const nav_msgs::Odometry::ConstPtr &msg )
{
	output = msg->pose.pose.position.y;
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

void OdoPosZInput::uprerun()
{
        subscribe(topic_name, (int)(size_queue()()) );
}

void OdoPosZInput::callback(const nav_msgs::Odometry::ConstPtr &msg )
{
	output = msg->pose.pose.position.z;
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
        subscribe(topic_name, (int)(size_queue()()) );
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

void OdoEulerRollInput::uprerun()
{
        subscribe(topic_name, (int)(size_queue()()) );
}

void OdoEulerRollInput::callback(const nav_msgs::Odometry::ConstPtr &msg )
{
	tf::Quaternion q(msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z,msg->pose.pose.orientation.w);

        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        output = roll;
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

void OdoEulerPitchInput::uprerun()
{
        subscribe(topic_name, (int)(size_queue()()) );
}

void OdoEulerPitchInput::callback(const nav_msgs::Odometry::ConstPtr &msg )
{
	tf::Quaternion q(msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z,msg->pose.pose.orientation.w);

        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        output = pitch;
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

void OdoEulerYawInput::uprerun()
{
        subscribe(topic_name, (int)(size_queue()()) );
}

void OdoEulerYawInput::callback(const nav_msgs::Odometry::ConstPtr &msg )
{
	tf::Quaternion q(msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z,msg->pose.pose.orientation.w);

        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        output = yaw;
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
        subscribe(topic_name, (int)(size_queue()()) );
}

void OdoQuaterInput::callback( const nav_msgs::Odometry::ConstPtr &msg )
{
        auto mout = getMapVect(output);
        mout[0] = msg->pose.pose.orientation.x;
        mout[1] = msg->pose.pose.orientation.y;
        mout[2] = msg->pose.pose.orientation.z;
        mout[3] = msg->pose.pose.orientation.w;
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

void OdoQuaterXInput::uprerun()
{
        subscribe(topic_name, (int)(size_queue()()) );
}

void OdoQuaterXInput::callback(const nav_msgs::Odometry::ConstPtr &msg )
{
        output = msg->pose.pose.orientation.x;
}

/*******************************************************************************************************/
/*********************                     OdoQuaterZInput                         ********************/
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

void OdoQuaterYInput::uprerun()
{
        subscribe(topic_name, (int)(size_queue()()) );
}

void OdoQuaterYInput::callback( const nav_msgs::Odometry::ConstPtr &msg )
{
        output = msg->pose.pose.orientation.y;
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

void OdoQuaterZInput::uprerun()
{
        subscribe(topic_name, (int)(size_queue()()) );
}

void OdoQuaterZInput::callback( const nav_msgs::Odometry::ConstPtr &msg )
{
        output = msg->pose.pose.orientation.z;
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

void OdoQuaterWInput::uprerun()
{
        subscribe(topic_name, (int)(size_queue()()) );
}

void OdoQuaterWInput::callback(const nav_msgs::Odometry::ConstPtr &msg )
{
        output = msg->pose.pose.orientation.w;
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
        subscribe(topic_name, (int)(size_queue()()) );
}

void OdoTwistLinInput::callback(const nav_msgs::Odometry::ConstPtr &msg )
{
        auto mout = getMapVect(output);
        mout[0] = msg->twist.twist.linear.x;
        mout[1] = msg->twist.twist.linear.y;
        mout[2] = msg->twist.twist.linear.z;
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

void OdoTwistLinXInput::uprerun()
{
        subscribe(topic_name, (int)(size_queue()()) );
}

void OdoTwistLinXInput::callback(const nav_msgs::Odometry::ConstPtr &msg )
{
        output = msg->twist.twist.linear.x;
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

void OdoTwistLinYInput::uprerun()
{
        subscribe(topic_name, (int)(size_queue()()) );
}

void OdoTwistLinYInput::callback(const nav_msgs::Odometry::ConstPtr &msg )
{
        output = msg->twist.twist.linear.y;
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

void OdoTwistLinZInput::uprerun()
{
        subscribe(topic_name, (int)(size_queue()()) );
}

void OdoTwistLinZInput::callback(const nav_msgs::Odometry::ConstPtr &msg )
{
        output = msg->twist.twist.linear.z;
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
        subscribe(topic_name, (int)(size_queue()()) );
}

void OdoTwistAngInput::callback(const nav_msgs::Odometry::ConstPtr &msg )
{
        auto mout = getMapVect(output);
        mout[0] = msg->twist.twist.angular.x;
        mout[1] = msg->twist.twist.angular.y;
        mout[2] = msg->twist.twist.angular.z;
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

void OdoTwistAngRollInput::uprerun()
{
        subscribe(topic_name, (int)(size_queue()()) );
}

void OdoTwistAngRollInput::callback(const nav_msgs::Odometry::ConstPtr &msg )
{
        output = msg->twist.twist.angular.x;
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

void OdoTwistAngPitchInput::uprerun()
{
        subscribe(topic_name, (int)(size_queue()()) );
}

void OdoTwistAngPitchInput::callback(const nav_msgs::Odometry::ConstPtr &msg )
{
        output = msg->twist.twist.angular.y;
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

void OdoTwistAngYawInput::uprerun()
{
        subscribe(topic_name, (int)(size_queue()()) );
}

void OdoTwistAngYawInput::callback(const nav_msgs::Odometry::ConstPtr &msg )
{
        output = msg->twist.twist.angular.z;
}


