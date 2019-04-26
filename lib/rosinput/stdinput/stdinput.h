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

#ifndef __STD_INPUT_HPP__
#define __STD_INPUT_HPP__

#include "kheops/kernel/function.h"
#include "kheops/kernel/kernel.h"
#include "kheops/ros/rossubscriber.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"

/*******************************************************************************************************/
/*****************             Kheops Basic Type Input (Scalar and Matrix)           *******************/
/*******************************************************************************************************/

/*
 * ScalarInput : ROS Input for SCALAR data
 */
class ScalarInput : public FScalar , public RosSubscriber<std_msgs::Float64>
{
        private :

                IString topic_name;
                ISInput size_queue;
                ISInput sleep;


        public :

                ScalarInput() :  RosSubscriber<std_msgs::Float64>(){}
                virtual ~ScalarInput(){}

                virtual void compute();
                virtual void setparameters();

                virtual void onQuit();
                virtual void onRun();
                virtual void onPause();
                virtual void callback(const std_msgs::Float64::ConstPtr &msg);
};

/*
 * MatrixInput : ROS Input for MATRIX data
 * Float64MultiArray should have the same dimension than the Output Matrix
 */
class MatrixInput : public FMatrix , public RosSubscriber<std_msgs::Float64MultiArray>
{
        private :

                IString topic_name;
                ISInput size_queue;
                ISInput sleep;

        public :

                MatrixInput() :  RosSubscriber<std_msgs::Float64MultiArray>()  {}
                virtual ~MatrixInput(){}

                virtual void compute();
                virtual void setparameters();

                virtual void onQuit();
                virtual void onRun();
                virtual void onPause();
                virtual void callback( const std_msgs::Float64MultiArray::ConstPtr &msg );
};


#endif // __STD_INPUT_HPP__
