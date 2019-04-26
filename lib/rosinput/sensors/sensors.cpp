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


#include "sensors.h"

REGISTER_FUNCTION(JoyAxes);
REGISTER_FUNCTION(JoyAxe);
REGISTER_FUNCTION(JoyButtons);
REGISTER_FUNCTION(JoyButton);
REGISTER_FUNCTION(LaserScan);

/*******************************************************************************************************/
/*****************                           JoyAxes                            *******************/
/*******************************************************************************************************/

void JoyAxes::compute()
{
        callOne(sleep()());
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
        callOne(sleep()());
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
        callOne(sleep()());
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
        callOne(sleep()());
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
/************************                       LaserScan                        *************************/
/*******************************************************************************************************/

void LaserScan::compute()
{
        callOne(sleep()());
}

void LaserScan::setparameters()
{
        moy.assign(output.size(), 0);

        Kernel::iBind(topic_name,"topic_name", getUuid());
        Kernel::iBind(size_queue,"size_queue", getUuid());
        Kernel::iBind(sleep,"sleep", getUuid());
        Kernel::iBind(range_max,"range_max", getUuid());
}

void LaserScan::callback(const sensor_msgs::LaserScan::ConstPtr &msg )
{
        auto mout = getMapVect(output);
        SCALAR RM = std::min(range_max()(),  (SCALAR)(msg->range_max) );
        SCALAR offset =  M_PI - fabs(msg->angle_min);

        for( unsigned int i = 0 ; i <  msg->ranges.size() ; i++)
        {
                unsigned int j = ( i * (msg->angle_max - msg->angle_min) /  msg->ranges.size() + offset ) * ( mout.size() / (2* M_PI)) ;

                SCALAR value = 1 - (msg->ranges[i] - msg->range_min) / (RM - msg->range_min) ;
                if( value < 0 ) value = 0;

                                                                if( j < mout.size() ) {
                        if( moy[j] == 0 ) mout(j) = value;
                        else mout(j) += value;

                        moy[j]++;
                                                                }
        }

        for(unsigned int i = 0 ; i < mout.size() ; i++)
        {
                if( moy[i] > 0 ) mout(i) = mout(i) / moy[i];
                else mout(i) = 0;

                moy[i] = 0;
        }
}

void LaserScan::onQuit()
{
        disable();
}

void LaserScan::onPause()
{
        disable();
}

void LaserScan::onRun()
{
        enable( topic_name, (int)(size_queue()())   );
}

