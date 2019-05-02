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

#include "std_sub.h"

REGISTER_FUNCTION(ScalarSub);
REGISTER_FUNCTION(MatrixSub);

/*******************************************************************************************************/
/*****************                             ScalarSub                           *******************/
/*******************************************************************************************************/

void ScalarSub::callback(const std_msgs::Float64::ConstPtr &msg)
{
        output = msg->data;
}

/*******************************************************************************************************/
/*****************                             MatrixSub                           *******************/
/*******************************************************************************************************/

void MatrixSub::callback( const std_msgs::Float64MultiArray::ConstPtr &msg)
{
        if( msg->layout.dim[0].size != output.rows() ||  msg->layout.dim[1].size !=  output.cols() )
        {
                throw std::invalid_argument("MatrixSub : Output dimension is not egal to the Float64MultiArray dimensions !");
        }

        Map<const MATRIX> mEnc (msg->data.data() , msg->layout.dim[0].size , msg->layout.dim[1].size );

        output = mEnc;
}
