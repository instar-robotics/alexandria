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


#ifndef __ROS_OUTPUT_HPP__
#define __ROS_OUTPUT_HPP__

#include "kheops/kernel/function.h"
#include "kheops/kernel/kernel.h"
#include <geometry_msgs/Accel.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

/*
 * CmdVelRawOutput : Send command velocity in geometry_msgs/Twist message 
 * 6 Scalars input (lin.x, lin.y, lin.z and rot.x, rot.y, rot.z)
 */
class CmdVelRawOutput : public FMatrix
{
	private :

		IString topic_name;
                ISInput size_queue;
		ISInput linX;
		ISInput linY;
		ISInput linZ;
		ISInput rotX;
		ISInput rotY;
		ISInput rotZ;

		ros::Publisher pub;

	public :
		
		CmdVelRawOutput(){}
		virtual ~CmdVelRawOutput(){}

		virtual void prerun();
		virtual void compute();
                virtual void setparameters();
};

/*
 * CmdVelVectOutput : Send command velocity in geometry_msgs/Twist message
 * 2 Vectors input (lin(x,y,z) and rot(x,y,z) where x = roll, y = pitch and z = yaw )
 */
class CmdVelVectOutput : public FMatrix
{
        private :

                IString topic_name;
                ISInput size_queue;
                ISMInput lin;
                ISMInput rot;

		ros::Publisher pub;

        public :

                CmdVelVectOutput(){}
                virtual ~CmdVelVectOutput(){}

                virtual void prerun();
                virtual void compute();
                virtual void setparameters();
};

/*
 * CmdVel2DOutput : Send command velocity in geometry_msgs/Twist message
 * 2 Scalar input (lin(x) and rot(z) where z = yaw )
 * This is the Function to command 2D Mobile Base
 */
class CmdVel2DOutput : public FMatrix
{
        private :

                IString topic_name;
                ISInput size_queue;
                ISInput lin;
                ISInput rot;
		
		ros::Publisher pub;

        public :

                CmdVel2DOutput(){}
                virtual ~CmdVel2DOutput(){}

                virtual void prerun();
                virtual void compute();
                virtual void setparameters();
};


/*
 * AccelRawOutput : Send acceleration parameters in geometry_msgs/Accel message 
 * 6 Scalars input (lin.x, lin.y, lin.z and rot.x, rot.y, rot.z)
 */
class AccelRawOutput : public FMatrix
{
        private :

                IString topic_name;
                ISInput size_queue;
                ISInput linX;
                ISInput linY;
                ISInput linZ;
                ISInput rotX;
                ISInput rotY;
                ISInput rotZ;

                ros::Publisher pub;

        public :

                AccelRawOutput(){}
                virtual ~AccelRawOutput(){}

                virtual void prerun();
                virtual void compute();
                virtual void setparameters();
};


/*
 * AccelVectOutput : Send acceleration parameters in geometry_msgs/Accel message 
 * 2 Vectors input (lin(x,y,z) and rot(x,y,z) where x = roll, y = pitch and z = yaw )
 */
class AccelVectOutput : public FMatrix
{
        private :

                IString topic_name;
                ISInput size_queue;
                ISMInput lin;
                ISMInput rot;

                ros::Publisher pub;

        public :

                AccelVectOutput(){}
                virtual ~AccelVectOutput(){}

                virtual void prerun();
                virtual void compute();
                virtual void setparameters();
};

/*
 * Accel2DOutput : Send acceleration parameters in geometry_msgs/Accel message
 * 2 Scalar input (lin(x) and rot(z) where z = yaw )
 * This is the Function to command 2D Mobile Base
 */
class Accel2DOutput : public FMatrix
{
        private :

                IString topic_name;
                ISInput size_queue;
                ISInput lin;
                ISInput rot;

                ros::Publisher pub;

        public :

                Accel2DOutput(){}
                virtual ~Accel2DOutput(){}

                virtual void prerun();
                virtual void compute();
                virtual void setparameters();
};

#endif // __ROS_OUTPUT_HPP__
