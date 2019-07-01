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

#ifndef __GEOMETRY_MSGS_HPP__
#define __GEOMETRY_MSGS_HPP__

#include "kheops/ros/fsub.h"
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Accel.h>
#include <geometry_msgs/Twist.h>

/* Note :
 *	1- Each FMatrixSub or FScalarSub object has 3 default Kheops Input:
 *	- IString topic_name : for the topic Name
 *	- ISInput size_queue : define the size of the queue
 *	- ISInput sleep      : define the behavior of the Function [blocking Function if sleep < 0 or non-blocking Function and time to wait if sleep >= 0]
 *
 *	2- This 3 inputs MUST BE BIND to the kernel in the method setparameters.
 *	If you extend setparameters to add other Inputs, don't forget to call FMatrixSub::setparameters or FScalarSub::setparameters ! 
 *
 *	3- And Most important : don't forget to add this Inputs in the XML description !
 *	For now, we don't have mechanisms to load automatically the input in the XML description
 *	Using XML ENTITY could be a good way to do this.
 */

/*******************************************************************************************************/
/*****************                             Vector3Sub                          *******************/
/*******************************************************************************************************/

/*
 * Vector3Sub: ROS Subscriber to represent a vector in free space. 
 */
class Vector3Sub: public FMatrixSub<geometry_msgs::Vector3>
{
        public :
                Vector3Sub() : FMatrixSub<geometry_msgs::Vector3>(VECTOR) {}
                virtual ~Vector3Sub(){}

                virtual void setparameters();
                virtual void callback(const geometry_msgs::Vector3::ConstPtr &msg);
};

/*
 * Vector3XSub: ROS Sub to represent a vector in free space. 
 * Get X component
 */
class Vector3XSub: public FScalarSub<geometry_msgs::Vector3>
{
        public :
                Vector3XSub() : FScalarSub<geometry_msgs::Vector3>() {}
                virtual ~Vector3XSub(){}

                virtual void callback( const geometry_msgs::Vector3::ConstPtr &msg );
};

/*
 * Vector3YSub: ROS Sub to represent a vector in free space. 
 * Get Y component
 */
class Vector3YSub: public FScalarSub<geometry_msgs::Vector3>
{
        public :
                Vector3YSub() : FScalarSub<geometry_msgs::Vector3>() {}
                virtual ~Vector3YSub(){}

                virtual void callback( const geometry_msgs::Vector3::ConstPtr &msg );
};

/*
 * Vector3ZSub: ROS Sub to represent a vector in free space. 
 * Get Z component
 */
class Vector3ZSub: public FScalarSub<geometry_msgs::Vector3>
{
        public :
                Vector3ZSub() : FScalarSub<geometry_msgs::Vector3>() {}
                virtual ~Vector3ZSub(){}

                virtual void callback( const geometry_msgs::Vector3::ConstPtr &msg );
};

/*******************************************************************************************************/
/******************                              AccelInput                          *******************/
/*******************************************************************************************************/

/*
 * AccelSub : This expresses acceleration in free space broken into its linear and angular parts.
 */
class AccelSub : public FMatrixSub<geometry_msgs::Accel>
{
        public :
                AccelSub() : FMatrixSub<geometry_msgs::Accel>(VECTOR) {}
                virtual ~AccelSub(){}

                virtual void setparameters();
                virtual void callback(const geometry_msgs::Accel::ConstPtr &msg);
};

/*
 * AccelLinearSub : This expresses acceleration in free space broken into its linear and angular parts.
 * Get the Linear part
 */
class AccelLinearSub : public FMatrixSub<geometry_msgs::Accel>
{
        public :
                AccelLinearSub() : FMatrixSub<geometry_msgs::Accel>(VECTOR) {}
                virtual ~AccelLinearSub(){}

                virtual void setparameters();
                virtual void callback(const geometry_msgs::Accel::ConstPtr &msg);
};

/*
 * AccelAngularSub : This expresses acceleration in free space broken into its linear and angular parts.
 * Get the Angular part
 */
class AccelAngularSub : public FMatrixSub<geometry_msgs::Accel>
{
        public :
                AccelAngularSub() : FMatrixSub<geometry_msgs::Accel>(VECTOR) {}
                virtual ~AccelAngularSub(){}

                virtual void setparameters();
                virtual void callback(const geometry_msgs::Accel::ConstPtr &msg);
};

/*******************************************************************************************************/
/******************                             TwistInput                           *******************/
/*******************************************************************************************************/

/*
 * TwistSub : This expresses velocity in free space broken into its linear and angular parts.
 */
class TwistSub : public FMatrixSub<geometry_msgs::Twist>
{
        public :
                TwistSub() : FMatrixSub<geometry_msgs::Twist>(VECTOR) {}
                virtual ~TwistSub(){}

                virtual void setparameters();
                virtual void callback(const geometry_msgs::Twist::ConstPtr &msg);
};

/*
 * TwistLinearSub : This expresses acceleration in free space broken into its linear and angular parts.
 * Get the Linear part
 */
class TwistLinearSub : public FMatrixSub<geometry_msgs::Twist>
{
        public :
                TwistLinearSub() : FMatrixSub<geometry_msgs::Twist>(VECTOR) {}
                virtual ~TwistLinearSub(){}

                virtual void setparameters();
                virtual void callback(const geometry_msgs::Twist::ConstPtr &msg);
};

/*
 * TwistAngularSub : This expresses acceleration in free space broken into its linear and angular parts.
 * Get the Angular part
 */
class TwistAngularSub : public FMatrixSub<geometry_msgs::Twist>
{
        public :
                TwistAngularSub() : FMatrixSub<geometry_msgs::Twist>(VECTOR) {}
                virtual ~TwistAngularSub(){}

                virtual void setparameters();
                virtual void callback(const geometry_msgs::Twist::ConstPtr &msg);
};

#endif //__GEOMETRY_MSGS_HPP__
