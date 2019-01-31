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
