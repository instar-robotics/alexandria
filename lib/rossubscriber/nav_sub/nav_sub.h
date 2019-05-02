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

#ifndef __NAV_MSGS_HPP__
#define __NAV_MSGS_HPP__

#include "kheops/ros/fsub.h"
#include <nav_msgs/Odometry.h>

/* Note :
 *      1- Each FMatrixSub or FScalarSub object has 3 default Kheops Input:
 *      - IString topic_name : for the topic Name
 *      - ISInput size_queue : define the size of the queue
 *      - ISInput sleep      : define the behavior of the Function [blocking Function if sleep < 0 or non-blocking Function and time to wait if sleep >= 0]
 *
 *      2- This 3 inputs MUST BE BIND to the kernel in the method setparameters.
 *      If you extend setparameters to add other Inputs, don't forget to call FMatrixSub::setparameters or FScalarSub::setparameters ! 
 *
 *      3- And Most important : don't forget to add this Inputs in the XML description !
 *      For now, we don't have mechanisms to load automatically the input in the XML description
 *      Using XML ENTITY could be a good way to do this.
 */ 

/*******************************************************************************************************/
/*****************                              Odometry                             *******************/
/*******************************************************************************************************/

/*
 * OdoPosSub : ROS Subscriber Odometric Cartesian Position
 * Vector with X,Y,Z absolute coordinate
 */
class OdoPosSub :  public FMatrixSub<nav_msgs::Odometry>
{
        public :

                OdoPosSub() : FMatrixSub<nav_msgs::Odometry>() {}
                virtual ~OdoPosSub(){}

                virtual void setparameters();
                virtual void callback( const nav_msgs::Odometry::ConstPtr &msg );
};

/*
 * OdoPosXSub : ROS Subscriber Odometric Cartesian Position, X component
 * Point X
 */
class OdoPosXSub :  public FScalarSub<nav_msgs::Odometry>
{
        public :

                OdoPosXSub() : FScalarSub<nav_msgs::Odometry>(){}
                virtual ~OdoPosXSub(){}

                virtual void callback( const nav_msgs::Odometry::ConstPtr &msg );
};


/*
 * OdoPosYSub : ROS Subscriber Odometric Cartesian Position, Y component
 * Point Y
 */
class OdoPosYSub : public FScalarSub<nav_msgs::Odometry>
{
        public :

                OdoPosYSub() : FScalarSub<nav_msgs::Odometry>(){}
                virtual ~OdoPosYSub(){}

                virtual void callback( const nav_msgs::Odometry::ConstPtr &msg );
};

/*
 * OdoPosZSub : ROS Subscriber Odometric Cartesian Position, Z component
 * Point Z
 */
class OdoPosZSub : public FScalarSub<nav_msgs::Odometry>
{
        public :

                OdoPosZSub() : FScalarSub<nav_msgs::Odometry>(){}
                virtual ~OdoPosZSub(){}

                virtual void callback( const nav_msgs::Odometry::ConstPtr &msg );
};

/*
 * OdoEulerSub : ROS Subscriber Odometric Absolute Orientation in Euler coordinate (Roll, Pitch, Yaw)
 * Vector Roll, Pitch, Yaw
 */
class OdoEulerSub : public FMatrixSub<nav_msgs::Odometry>
{
        public :

                OdoEulerSub() : FMatrixSub<nav_msgs::Odometry>(){}
                virtual ~OdoEulerSub(){}

                virtual void setparameters();
                virtual void callback( const nav_msgs::Odometry::ConstPtr &msg );
};

/*
 * OdoEulerRollSub : ROS Subscriber Odometric Absolute Orientation in Euler coordinate (Roll)
 * Scalar Roll
 */
class OdoEulerRollSub : public FScalarSub<nav_msgs::Odometry>
{
        public :

                OdoEulerRollSub() : FScalarSub<nav_msgs::Odometry>(){}
                virtual ~OdoEulerRollSub(){}

                virtual void callback( const nav_msgs::Odometry::ConstPtr &msg );
};

/*
 * OdoEulerPitchSub : ROS Subscriber Odometric Absolute Orientation in Euler coordinate (Pitch)
 * Scalar Pitch
 */
class OdoEulerPitchSub : public FScalarSub<nav_msgs::Odometry>
{
        public :

                OdoEulerPitchSub() : FScalarSub<nav_msgs::Odometry>(){}
                virtual ~OdoEulerPitchSub(){}

                virtual void callback( const nav_msgs::Odometry::ConstPtr &msg );
};


/*
 * OdoEulerYawSub : ROS Subscriber Odometric Absolute Orientation in Euler coordinate (Yaw)
 * Scalar Yaw
 */
class OdoEulerYawSub : public FScalarSub<nav_msgs::Odometry>
{
        public :

                OdoEulerYawSub() : FScalarSub<nav_msgs::Odometry>(){}
                virtual ~OdoEulerYawSub(){}

                virtual void callback( const nav_msgs::Odometry::ConstPtr &msg );
};

/*
 * OdoQuaterSub : ROS Subscriber Odometric Absolute Orientation in Quaternion coordinate (X,Y,Z,W)
 * Vector X,Y,Z,W
 */
class OdoQuaterSub : public FMatrixSub<nav_msgs::Odometry>
{
        public :

                OdoQuaterSub() : FMatrixSub<nav_msgs::Odometry>(){}
                virtual ~OdoQuaterSub(){}

                virtual void setparameters();
                virtual void callback( const nav_msgs::Odometry::ConstPtr &msg );
};


/*
 * OdoQuaterXSub : ROS Subscriber Odometric Absolute Orientation in Quaternion coodinate (X)
 * Scalar X
 */
class OdoQuaterXSub : public FScalarSub<nav_msgs::Odometry>
{
        public :

                OdoQuaterXSub() : FScalarSub<nav_msgs::Odometry>(){}
                virtual ~OdoQuaterXSub(){}

                virtual void callback( const nav_msgs::Odometry::ConstPtr &msg );
};


/*
 * OdoQuaterYSub : ROS Subscriber Odometric Absolute Orientation in Quaternion coodinate (Y)
 * Scalar Y
 */
class OdoQuaterYSub : public FScalarSub<nav_msgs::Odometry>
{
        public :

                OdoQuaterYSub() : FScalarSub<nav_msgs::Odometry>(){}
                virtual ~OdoQuaterYSub(){}

                virtual void callback( const nav_msgs::Odometry::ConstPtr &msg );
};

/*
 * OdoQuaterZSub : ROS Subscriber Odometric Absolute Orientation in Quaternion coodinate (Z)
 * Scalar Z
 */
class OdoQuaterZSub : public FScalarSub<nav_msgs::Odometry>
{
        public :

                OdoQuaterZSub() : FScalarSub<nav_msgs::Odometry>(){}
                virtual ~OdoQuaterZSub(){}

                virtual void callback( const nav_msgs::Odometry::ConstPtr &msg );
};

/*
 * OdoQuaterWSub : ROS Subscriber Odometric Absolute Orientation in Quaternion coodinate (W)
 * Scalar W
 */
class OdoQuaterWSub : public FScalarSub<nav_msgs::Odometry>
{
        public :

                OdoQuaterWSub() : FScalarSub<nav_msgs::Odometry>(){}
                virtual ~OdoQuaterWSub(){}

                virtual void callback( const nav_msgs::Odometry::ConstPtr &msg );
};


/*
 * OdoTwistLinSub : ROS Subscriber Odometric Twist, Linear movement on (X,Y,Z)
 * Vector (X,Y,Z)
 */
class OdoTwistLinSub : public FMatrixSub<nav_msgs::Odometry>
{
        public :

                OdoTwistLinSub() : FMatrixSub<nav_msgs::Odometry>(){}
                virtual ~OdoTwistLinSub(){}

                virtual void setparameters();
                virtual void callback( const nav_msgs::Odometry::ConstPtr &msg );
};

/*
 * OdoTwistLinXSub : ROS Subscriber Odometric Twist, Linear movement on X
 * Scalar X
 */
class OdoTwistLinXSub : public FScalarSub<nav_msgs::Odometry>
{
        public :

                OdoTwistLinXSub() : FScalarSub<nav_msgs::Odometry>(){}
                virtual ~OdoTwistLinXSub(){}

                virtual void callback( const nav_msgs::Odometry::ConstPtr &msg );
};


/*
 * OdoTwistLinXSub : ROS Subscriber Odometric Twist, Linear movement on X
 * Scalar Y
 */
class OdoTwistLinYSub : public FScalarSub<nav_msgs::Odometry>
{
        public :

                OdoTwistLinYSub() : FScalarSub<nav_msgs::Odometry>(){}
                virtual ~OdoTwistLinYSub(){}

                virtual void callback( const nav_msgs::Odometry::ConstPtr &msg );
};

/*
 * OdoTwistLinXSub : ROS Subscriber Odometric Twist, Linear movement on X
 * Scalar Z
 */
class OdoTwistLinZSub : public FScalarSub<nav_msgs::Odometry>
{
        public :

                OdoTwistLinZSub() : FScalarSub<nav_msgs::Odometry>(){}
                virtual ~OdoTwistLinZSub(){}

                virtual void callback( const nav_msgs::Odometry::ConstPtr &msg );
};

/*
 * OdoTwistAngSub : ROS Subscriber Odometric Twist, Angular movement on (Roll,Pitch,Yaw)
 * Vector (Roll, Pitch, Yaw)
 */
class OdoTwistAngSub : public FMatrixSub<nav_msgs::Odometry>
{
        public :

                OdoTwistAngSub() : FMatrixSub<nav_msgs::Odometry>(){}
                virtual ~OdoTwistAngSub(){}

                virtual void setparameters();
                virtual void callback( const nav_msgs::Odometry::ConstPtr &msg );
};

/*
 * OdoTwistAngRollSub : ROS Subscriber Odometric Twist, Angular movement on (Roll)
 * Scalar (Roll)
 */
class OdoTwistAngRollSub : public FScalarSub<nav_msgs::Odometry>
{
        public :

                OdoTwistAngRollSub() : FScalarSub<nav_msgs::Odometry>(){}
                virtual ~OdoTwistAngRollSub(){}

                virtual void callback( const nav_msgs::Odometry::ConstPtr &msg );
};

/*
 * OdoTwistAngPitchSub : ROS Subscriber Odometric Twist, Angular movement on (Pitch)
 * Scalar (Pitch)
 */
class OdoTwistAngPitchSub : public FScalarSub<nav_msgs::Odometry>
{
        public :

                OdoTwistAngPitchSub() : FScalarSub<nav_msgs::Odometry>(){}
                virtual ~OdoTwistAngPitchSub(){}

                virtual void callback( const nav_msgs::Odometry::ConstPtr &msg );
};

/*
 * OdoTwistAngYawSub : ROS Subscriber Odometric Twist, Angular movement on (Yaw)
 * Scalar (Yaw)
 */
class OdoTwistAngYawSub : public FScalarSub<nav_msgs::Odometry>
{
        public :

                OdoTwistAngYawSub() : FScalarSub<nav_msgs::Odometry>(){}
                virtual ~OdoTwistAngYawSub(){}

                virtual void callback( const nav_msgs::Odometry::ConstPtr &msg );
};

#endif // __NAV_MSGS_HPP__
