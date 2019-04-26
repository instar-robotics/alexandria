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

#include "stdinput.h"

REGISTER_FUNCTION(ScalarInput);
REGISTER_FUNCTION(MatrixInput);

/*******************************************************************************************************/
/*****************                             ScalarInput                           *******************/
/*******************************************************************************************************/

void ScalarInput::compute()
{
        callOne(sleep()());
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
        callOne(sleep()());
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

        Map<const MATRIX> mEnc (msg->data.data() , msg->layout.dim[0].size , msg->layout.dim[1].size );

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

