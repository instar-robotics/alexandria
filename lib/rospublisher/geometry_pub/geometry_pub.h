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


#ifndef __GEOMETRY_PUB_HPP__
#define __GEOMETRY_PUB_HPP__

#include "kheops/ros/fpub.h"
#include <geometry_msgs/Accel.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

/* Note :
 *      1- Each FMatrixPub or FScalarPub object has 2 default Kheops Input:
 *      - IString topic_name : for the topic Name
 *      - ISInput size_queue : define the size of the queue
 *
 *      2- This 2 inputs MUST BE BIND to the kernel in the method setparameters.
 *      If you extend setparameters to add other Inputs, don't forget to call FMatrixPub::setparameters or FScalarPub::setparameters !
 *
 *      3- And Most important : don't forget to add this Inputs in the XML description !
 *      For now, we don't have mechanisms to load automatically the input in the XML description
 *      Using XML ENTITY could be a good way to do this.
 */


/*
 * Vector3SPub : ROS publisher takes 3 Scalar to publish a Vector3 message
 */
class Vector3SPub : public  FMatrixPub<geometry_msgs::Vector3>
{
	private : 

		ISInput x;
		ISInput y;
		ISInput z;
	public :

                Vector3SPub() : FMatrixPub<geometry_msgs::Vector3>(VECTOR) {}
                virtual ~Vector3SPub(){}

                virtual void compute();
                virtual void setparameters();
};

/*
 * Vector3Pub : ROS publisher publish a Vector3 message
 */
class Vector3Pub : public  FMatrixPub<geometry_msgs::Vector3>
{
	private : 

		ISMInput inVector;

	public :

                Vector3Pub() : FMatrixPub<geometry_msgs::Vector3>(VECTOR) {}
                virtual ~Vector3Pub(){}

                virtual void compute();
		virtual void prerun();
                virtual void setparameters();
};

/*
 * TwistPub : Send command velocity in geometry_msgs/Twist message 
 * 6 Scalars input (lin.x, lin.y, lin.z and rot.x, rot.y, rot.z)
 */
class TwistPub : public FMatrixPub<geometry_msgs::Twist>
{
	private :

		ISInput linX;
		ISInput linY;
		ISInput linZ;
		ISInput rotX;
		ISInput rotY;
		ISInput rotZ;

	public :
		
		TwistPub() : FMatrixPub<geometry_msgs::Twist>(VECTOR) {}
		virtual ~TwistPub(){}

		virtual void compute();
                virtual void setparameters();
};

/*
 * TwistVectPub: Send command velocity in geometry_msgs/Twist message
 * 2 Vectors input (lin(x,y,z) and rot(x,y,z) where x = roll, y = pitch and z = yaw )
 */
class TwistVectPub : public FMatrixPub<geometry_msgs::Twist>
{
        private :

                ISMInput lin;
                ISMInput rot;

        public :

                TwistVectPub() : FMatrixPub<geometry_msgs::Twist>(VECTOR) {}
                virtual ~TwistVectPub(){}

                virtual void compute();
		virtual void prerun();
                virtual void setparameters();
};

/*
 * Twist2DPub : Send command velocity in geometry_msgs/Twist message
 * 2 Scalar input (lin(x) and rot(z) where z = yaw )
 * This is the Function to command 2D Mobile Base
 */
class Twist2DPub : public FMatrixPub<geometry_msgs::Twist> 
{
        private :

                ISInput lin;
                ISInput rot;
		
        public :

                Twist2DPub() : FMatrixPub<geometry_msgs::Twist>(VECTOR) {}
                virtual ~Twist2DPub(){}

                virtual void compute();
                virtual void setparameters();
};


/*
 * AccelPub : Send acceleration parameters in geometry_msgs/Accel message 
 * 6 Scalars input (lin.x, lin.y, lin.z and rot.x, rot.y, rot.z)
 */
class AccelPub : public FMatrixPub<geometry_msgs::Accel>
{
        private :

                ISInput linX;
                ISInput linY;
                ISInput linZ;
                ISInput rotX;
                ISInput rotY;
                ISInput rotZ;

        public :

                AccelPub() : FMatrixPub<geometry_msgs::Accel>(VECTOR)  {}
                virtual ~AccelPub(){}

                virtual void compute();
                virtual void setparameters();
};


/*
 * AccelVectPub : Send acceleration parameters in geometry_msgs/Accel message 
 * 2 Vectors input (lin(x,y,z) and rot(x,y,z) where x = roll, y = pitch and z = yaw )
 */
class AccelVectPub : public FMatrixPub<geometry_msgs::Accel>
{
        private :

                ISMInput lin;
                ISMInput rot;

        public :

                AccelVectPub() : FMatrixPub<geometry_msgs::Accel>(VECTOR) {}
                virtual ~AccelVectPub(){}

                virtual void prerun();
                virtual void compute();
                virtual void setparameters();
};

/*
 * Accel2DPub : Send acceleration parameters in geometry_msgs/Accel message
 * 2 Scalar input (lin(x) and rot(z) where z = yaw )
 * This is the Function to command 2D Mobile Base
 */
class Accel2DPub : public FMatrixPub<geometry_msgs::Accel>
{
        private :

                ISInput lin;
                ISInput rot;

        public :

                Accel2DPub() : FMatrixPub<geometry_msgs::Accel>(VECTOR){}
                virtual ~Accel2DPub(){}

                virtual void compute();
                virtual void setparameters();
};

/*
 * Pose : Send pose parameters in geometry_msgs/Pose
 * position : a 3D Vector
 * orientation : a 3D Vector (Euler angle, will be convert to Quaternion)
 * Output dimension should be 6 (3 Scalar for pos and 3 for orientation)
 */
class PosePub :  public FMatrixPub<geometry_msgs::Pose>
{
        private :

                ISMInput position;
                ISMInput orientation;

                geometry_msgs::Pose msg;

        public :

                PosePub() : FMatrixPub<geometry_msgs::Pose>(VECTOR) {}
                virtual ~PosePub(){}

                virtual void compute();
                virtual void setparameters();
                virtual void prerun();
};


/*
 * PoseStamped : Send pose parameters in geometry_msgs/PoseStamped
 * position : a 3D Vector
 * orientation : a 3D Vector (Euler angle, will be convert to Quaternion)
 * Output dimension should be 6 (3 Scalar for pos and 3 for orientation)
 */
class PoseStampedPub :  public FMatrixPub<geometry_msgs::PoseStamped>
{
	private :

		IString frame_id;
                ISMInput position;
                ISMInput orientation;

		geometry_msgs::PoseStamped msg;

	public :

                PoseStampedPub() : FMatrixPub<geometry_msgs::PoseStamped>(VECTOR) {}
                virtual ~PoseStampedPub(){}

                virtual void compute();
                virtual void setparameters();
		virtual void prerun();
};

/*
 * PointStamped : Send pose parameters in geometry_msgs/PointStamped
 * position : a 3D Vector
 * Output dimension should be 2 (3 Scalar for point)
 */
class PointStampedPub :  public FMatrixPub<geometry_msgs::PointStamped>
{
        private :

                IString frame_id;
                ISMInput point;

                geometry_msgs::PointStamped msg;

        public :

                PointStampedPub() : FMatrixPub<geometry_msgs::PointStamped>(VECTOR) {}
                virtual ~PointStampedPub(){}

                virtual void compute();
                virtual void setparameters();
                virtual void prerun();
};


#endif // __ROS_OUTPUT_HPP__
