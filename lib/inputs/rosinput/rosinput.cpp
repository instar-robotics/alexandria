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

REGISTER_FUNCTION(ScalarInput);
REGISTER_FUNCTION(MatrixInput);
REGISTER_FUNCTION(JoyAxesInput);
REGISTER_FUNCTION(JoyAxeInput);
REGISTER_FUNCTION(JoyButtonsInput);
REGISTER_FUNCTION(JoyButtonInput);

/********************************************************************************************************/
/******************                             ScalarInput                           *******************/
/********************************************************************************************************/

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

/********************************************************************************************************/
/******************                             MatrixInput                           *******************/
/********************************************************************************************************/

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

 	Map<const MatrixXd> mEnc ( msg->data.data() , msg->layout.dim[0].size , msg->layout.dim[1].size );

       	output = mEnc;
}


/********************************************************************************************************/
/******************                           JoyAxesInput                            *******************/
/********************************************************************************************************/

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

/********************************************************************************************************/
/******************                           JoyAxeInput                            *******************/
/********************************************************************************************************/

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


/********************************************************************************************************/
/******************                         JoyButtonsInput                           *******************/
/********************************************************************************************************/

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


/********************************************************************************************************/
/******************                           JoyButtonInput                          *******************/
/********************************************************************************************************/

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

