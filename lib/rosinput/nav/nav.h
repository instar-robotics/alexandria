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

#ifndef __NAV_HPP__
#define __NAV_HPP__

#include "kheops/kernel/function.h"
#include "kheops/kernel/kernel.h"
#include "kheops/ros/rossubscriber.h"
#include <nav_msgs/Odometry.h>

/*******************************************************************************************************/
/*****************                          Odometry Inputs                          *******************/
/*******************************************************************************************************/

/*
 * OdoPos : ROS Input Odometric Cartesian Position
 * Vector with X,Y,Z absolute coordinate
 */
class OdoPos :  public FMatrix, public RosSubscriber<nav_msgs::Odometry>
{
        private :

                IString topic_name;
                ISInput size_queue;
                ISInput sleep;

        public :

                OdoPos() : RosSubscriber<nav_msgs::Odometry>() {}
                virtual ~OdoPos(){}

                virtual void prerun();
                virtual void compute();
                virtual void setparameters();

                virtual void onQuit();
                virtual void onRun();
                virtual void onPause();
                virtual void callback( const nav_msgs::Odometry::ConstPtr &msg );
};

/*
 * OdoPosX : ROS Input Odometric Cartesian Position, X component
 * Point X
 */
class OdoPosX :  public FScalar, public RosSubscriber<nav_msgs::Odometry>
{
        private :

                IString topic_name;
                ISInput size_queue;
                ISInput sleep;

        public :

                OdoPosX() : RosSubscriber<nav_msgs::Odometry>(){}
                virtual ~OdoPosX(){}

                virtual void compute();
                virtual void setparameters();

                virtual void onQuit();
                virtual void onRun();
                virtual void onPause();
                virtual void callback( const nav_msgs::Odometry::ConstPtr &msg );
};


/*
 * OdoPosY : ROS Input Odometric Cartesian Position, Y component
 * Point Y
 */
class OdoPosY : public FScalar, public RosSubscriber<nav_msgs::Odometry>
{
        private :

                IString topic_name;
                ISInput size_queue;
                ISInput sleep;

        public :

                OdoPosY() : RosSubscriber<nav_msgs::Odometry>(){}
                virtual ~OdoPosY(){}

                virtual void compute();
                virtual void setparameters();

                virtual void onQuit();
                virtual void onRun();
                virtual void onPause();
                virtual void callback( const nav_msgs::Odometry::ConstPtr &msg );
};

/*
 * OdoPosZ : ROS Input Odometric Cartesian Position, Z component
 * Point Z
 */
class OdoPosZ : public FScalar, public RosSubscriber<nav_msgs::Odometry>
{
        private :

                IString topic_name;
                ISInput size_queue;
                ISInput sleep;

        public :

                OdoPosZ() : RosSubscriber<nav_msgs::Odometry>(){}
                virtual ~OdoPosZ(){}

                virtual void compute();
                virtual void setparameters();

                virtual void onQuit();
                virtual void onRun();
                virtual void onPause();
                virtual void callback( const nav_msgs::Odometry::ConstPtr &msg );
};

/*
 * OdoEuler : ROS Input Odometric Absolute Orientation in Euler coordinate (Roll, Pitch, Yaw)
 * Vector Roll, Pitch, Yaw
 */
class OdoEuler : public FMatrix, public RosSubscriber<nav_msgs::Odometry>
{
        private :

                IString topic_name;
                ISInput size_queue;
                ISInput sleep;

        public :

                OdoEuler() : RosSubscriber<nav_msgs::Odometry>(){}
                virtual ~OdoEuler(){}

                virtual void prerun();
                virtual void compute();
                virtual void setparameters();

                virtual void onQuit();
                virtual void onRun();
                virtual void onPause();
                virtual void callback( const nav_msgs::Odometry::ConstPtr &msg );
};



/*
 * OdoEulerRoll : ROS Input Odometric Absolute Orientation in Euler coordinate (Roll)
 * Scalar Roll
 */
class OdoEulerRoll : public FScalar, public RosSubscriber<nav_msgs::Odometry>
{
        private :

                IString topic_name;
                ISInput size_queue;
                ISInput sleep;

        public :

                OdoEulerRoll() : RosSubscriber<nav_msgs::Odometry>(){}
                virtual ~OdoEulerRoll(){}

                virtual void compute();
                virtual void setparameters();

                virtual void onQuit();
                virtual void onRun();
                virtual void onPause();
                virtual void callback( const nav_msgs::Odometry::ConstPtr &msg );
};

/*
 * OdoEulerPitch : ROS Input Odometric Absolute Orientation in Euler coordinate (Pitch)
 * Scalar Pitch
 */
class OdoEulerPitch : public FScalar, public RosSubscriber<nav_msgs::Odometry>
{
        private :

                IString topic_name;
                ISInput size_queue;
                ISInput sleep;

        public :

                OdoEulerPitch() : RosSubscriber<nav_msgs::Odometry>(){}
                virtual ~OdoEulerPitch(){}

                virtual void compute();
                virtual void setparameters();

                virtual void onQuit();
                virtual void onRun();
                virtual void onPause();
                virtual void callback( const nav_msgs::Odometry::ConstPtr &msg );
};


/*
 * OdoEulerYaw : ROS Input Odometric Absolute Orientation in Euler coordinate (Yaw)
 * Scalar Yaw
 */
class OdoEulerYaw : public FScalar, public RosSubscriber<nav_msgs::Odometry>
{
        private :

                IString topic_name;
                ISInput size_queue;
                ISInput sleep;

        public :

                OdoEulerYaw() : RosSubscriber<nav_msgs::Odometry>(){}
                virtual ~OdoEulerYaw(){}

                virtual void compute();
                virtual void setparameters();

                virtual void onQuit();
                virtual void onRun();
                virtual void onPause();
                virtual void callback( const nav_msgs::Odometry::ConstPtr &msg );
};

/*
 * OdoQuater : ROS Input Odometric Absolute Orientation in Quaternion coordinate (X,Y,Z,W)
 * Vector X,Y,Z,W
 */
class OdoQuater : public FMatrix, public RosSubscriber<nav_msgs::Odometry>
{
        private :
                IString topic_name;
                ISInput size_queue;
                ISInput sleep;

        public :

                OdoQuater() : RosSubscriber<nav_msgs::Odometry>(){}
                virtual ~OdoQuater(){}

                virtual void prerun();
                virtual void compute();
                virtual void setparameters();

                virtual void onQuit();
                virtual void onRun();
                virtual void onPause();
                virtual void callback( const nav_msgs::Odometry::ConstPtr &msg );
};


/*
 * OdoQuaterX : ROS Input Odometric Absolute Orientation in Quaternion coodinate (X)
 * Scalar X
 */
class OdoQuaterX : public FScalar, public RosSubscriber<nav_msgs::Odometry>
{
        private :

                IString topic_name;
                ISInput size_queue;
                ISInput sleep;

        public :

                OdoQuaterX() : RosSubscriber<nav_msgs::Odometry>(){}
                virtual ~OdoQuaterX(){}

                virtual void compute();
                virtual void setparameters();

                virtual void onQuit();
                virtual void onRun();
                virtual void onPause();
                virtual void callback( const nav_msgs::Odometry::ConstPtr &msg );
};


/*
 * OdoQuaterY : ROS Input Odometric Absolute Orientation in Quaternion coodinate (Y)
 * Scalar Y
 */
class OdoQuaterY : public FScalar, public RosSubscriber<nav_msgs::Odometry>
{
        private :

                IString topic_name;
                ISInput size_queue;
                ISInput sleep;

        public :

                OdoQuaterY() : RosSubscriber<nav_msgs::Odometry>(){}
                virtual ~OdoQuaterY(){}

                virtual void compute();
                virtual void setparameters();

                virtual void onQuit();
                virtual void onRun();
                virtual void onPause();
                virtual void callback( const nav_msgs::Odometry::ConstPtr &msg );
};

/*
 * OdoQuaterZ : ROS Input Odometric Absolute Orientation in Quaternion coodinate (Z)
 * Scalar Z
 */
class OdoQuaterZ : public FScalar, public RosSubscriber<nav_msgs::Odometry>
{
        private :

                IString topic_name;
                ISInput size_queue;
                ISInput sleep;

        public :

                OdoQuaterZ() : RosSubscriber<nav_msgs::Odometry>(){}
                virtual ~OdoQuaterZ(){}

                virtual void compute();
                virtual void setparameters();

                virtual void onQuit();
                virtual void onRun();
                virtual void onPause();
                virtual void callback( const nav_msgs::Odometry::ConstPtr &msg );
};

/*
 * OdoQuaterW : ROS Input Odometric Absolute Orientation in Quaternion coodinate (W)
 * Scalar W
 */
class OdoQuaterW : public FScalar, public RosSubscriber<nav_msgs::Odometry>
{
        private :

                IString topic_name;
                ISInput size_queue;
                ISInput sleep;

        public :

                OdoQuaterW() : RosSubscriber<nav_msgs::Odometry>(){}
                virtual ~OdoQuaterW(){}

                virtual void compute();
                virtual void setparameters();

                virtual void onQuit();
                virtual void onRun();
                virtual void onPause();
                virtual void callback( const nav_msgs::Odometry::ConstPtr &msg );
};


/*
 * OdoTwistLin : ROS Input Odometric Twist, Linear movement on (X,Y,Z)
 * Vector (X,Y,Z)
 */
class OdoTwistLin : public FMatrix, public RosSubscriber<nav_msgs::Odometry>
{
        private :

                IString topic_name;
                ISInput size_queue;
                ISInput sleep;

        public :

                OdoTwistLin() : RosSubscriber<nav_msgs::Odometry>(){}
                virtual ~OdoTwistLin(){}

                virtual void prerun();
                virtual void compute();
                virtual void setparameters();

                virtual void onQuit();
                virtual void onRun();
                virtual void onPause();
                virtual void callback( const nav_msgs::Odometry::ConstPtr &msg );
};

/*
 * OdoTwistLinX : ROS Input Odometric Twist, Linear movement on X
 * Scalar X
 */
class OdoTwistLinX : public FScalar, public RosSubscriber<nav_msgs::Odometry>
{
        private :

                IString topic_name;
                ISInput size_queue;
                ISInput sleep;

        public :

                OdoTwistLinX() : RosSubscriber<nav_msgs::Odometry>(){}
                virtual ~OdoTwistLinX(){}

                virtual void compute();
                virtual void setparameters();

                virtual void onQuit();
                virtual void onRun();
                virtual void onPause();
                virtual void callback( const nav_msgs::Odometry::ConstPtr &msg );
};


/*
 * OdoTwistLinX : ROS Input Odometric Twist, Linear movement on Y
 * Scalar Y
 */
class OdoTwistLinY : public FScalar, public RosSubscriber<nav_msgs::Odometry>
{
        private :

                IString topic_name;
                ISInput size_queue;
                ISInput sleep;

        public :

                OdoTwistLinY() : RosSubscriber<nav_msgs::Odometry>(){}
                virtual ~OdoTwistLinY(){}

                virtual void compute();
                virtual void setparameters();

                virtual void onQuit();
                virtual void onRun();
                virtual void onPause();
                virtual void callback( const nav_msgs::Odometry::ConstPtr &msg );

};

/*
 * OdoTwistLinZ : ROS Input Odometric Twist, Linear movement on Z
 * Scalar Z
 */
class OdoTwistLinZ : public FScalar, public RosSubscriber<nav_msgs::Odometry>
{
        private :

                IString topic_name;
                ISInput size_queue;
                ISInput sleep;

        public :

                OdoTwistLinZ() : RosSubscriber<nav_msgs::Odometry>(){}
                virtual ~OdoTwistLinZ(){}

                virtual void compute();
                virtual void setparameters();

                virtual void onQuit();
                virtual void onRun();
                virtual void onPause();
                virtual void callback( const nav_msgs::Odometry::ConstPtr &msg );
};

/*
 * OdoTwistAng : ROS Input Odometric Twist, Angular movement on (Roll,Pitch,Yaw)
 * Vector (Roll, Pitch, Yaw)
 */
class OdoTwistAng : public FMatrix, public RosSubscriber<nav_msgs::Odometry>
{
        private :

                IString topic_name;
                ISInput size_queue;
                ISInput sleep;

        public :

                OdoTwistAng() : RosSubscriber<nav_msgs::Odometry>(){}
                virtual ~OdoTwistAng(){}

                virtual void prerun();
                virtual void compute();
                virtual void setparameters();

                virtual void onQuit();
                virtual void onRun();
                virtual void onPause();
                virtual void callback( const nav_msgs::Odometry::ConstPtr &msg );

};

/*
 * OdoTwistAngRoll : ROS Input Odometric Twist, Angular movement on (Roll)
 * Scalar (Roll)
 */
class OdoTwistAngRoll : public FScalar, public RosSubscriber<nav_msgs::Odometry>
{
        private :

                IString topic_name;
                ISInput size_queue;
                ISInput sleep;

        public :

                OdoTwistAngRoll() : RosSubscriber<nav_msgs::Odometry>(){}
                virtual ~OdoTwistAngRoll(){}

                virtual void compute();
                virtual void setparameters();

                virtual void onQuit();
                virtual void onRun();
                virtual void onPause();
                virtual void callback( const nav_msgs::Odometry::ConstPtr &msg );
};

/*
 * OdoTwistAngPitch : ROS Input Odometric Twist, Angular movement on (Pitch)
 * Scalar (Pitch)
 */
class OdoTwistAngPitch : public FScalar, public RosSubscriber<nav_msgs::Odometry>
{
        private :

                IString topic_name;
                ISInput size_queue;
                ISInput sleep;

        public :

                OdoTwistAngPitch() : RosSubscriber<nav_msgs::Odometry>(){}
                virtual ~OdoTwistAngPitch(){}

                virtual void compute();
                virtual void setparameters();

                virtual void onQuit();
                virtual void onRun();
                virtual void onPause();
                virtual void callback( const nav_msgs::Odometry::ConstPtr &msg );
};

/*
 * OdoTwistAngYaw : ROS Input Odometric Twist, Angular movement on (Yaw)
 * Scalar (Yaw)
 */
class OdoTwistAngYaw : public FScalar, public RosSubscriber<nav_msgs::Odometry>
{
        private :

                IString topic_name;
                ISInput size_queue;
                ISInput sleep;

        public :

                OdoTwistAngYaw() : RosSubscriber<nav_msgs::Odometry>(){}
                virtual ~OdoTwistAngYaw(){}

                virtual void compute();
                virtual void setparameters();

                virtual void onQuit();
                virtual void onRun();
                virtual void onPause();
                virtual void callback( const nav_msgs::Odometry::ConstPtr &msg );
};

#endif // __NAV_HPP__
