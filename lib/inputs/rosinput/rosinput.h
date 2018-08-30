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

#ifndef __ROS_INPUT_HPP__
#define __ROS_INPUT_HPP__

#include "kheops/kernel/function.h"
#include "kheops/kernel/kernel.h"
#include "kheops/ros/rossubscriber.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/Joy.h"

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

		virtual void uprerun();
		virtual void compute();
                virtual void setparameters();
		
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

		virtual void uprerun();
		virtual void compute();
                virtual void setparameters();

		virtual void callback( const std_msgs::Float64MultiArray::ConstPtr &msg );

};

/*
 * JoyAxesInput : ROS Input for Joystick's axes values
 * Axes array must have the same dimension than the Output Matrix
 * But Matrix could be in any row/col form
 */
class JoyAxesInput : public FMatrix, public RosSubscriber<sensor_msgs::Joy>
{
	private :

		IString topic_name;
		ISInput size_queue;
		ISInput sleep;

	public : 
		JoyAxesInput() : RosSubscriber<sensor_msgs::Joy>() {}
		virtual ~JoyAxesInput(){}
		
		virtual void uprerun();
		virtual void compute();
                virtual void setparameters();

		virtual void callback( const sensor_msgs::Joy::ConstPtr &msg );
};

/*
 * JoyAxeInput : ROS Input for one Joystick's axe value
 * axe : the ID of the axe in the axes's array
 */
class JoyAxeInput : public FScalar, public RosSubscriber<sensor_msgs::Joy>
{
        private :

                IString topic_name;
                ISInput size_queue;
                ISInput sleep;
                ISInput axe;

        public :
                JoyAxeInput() : RosSubscriber<sensor_msgs::Joy>() {}
                virtual ~JoyAxeInput(){}

                virtual void uprerun();
                virtual void compute();
                virtual void setparameters();

                virtual void callback( const sensor_msgs::Joy::ConstPtr &msg );
};

/*
 * JoyButtonsInput : ROS Input for Joystick's buttons values
 * Buttons array must have the same dimension than the Output Matrix
 * But Matrix could be in any row/col form
 */
class JoyButtonsInput : public FMatrix, public RosSubscriber<sensor_msgs::Joy>
{
        private :

                IString topic_name;
                ISInput size_queue;
                ISInput sleep;

        public :
                JoyButtonsInput() : RosSubscriber<sensor_msgs::Joy>() {}
                virtual ~JoyButtonsInput(){}

                virtual void uprerun();
                virtual void compute();
                virtual void setparameters();

                virtual void callback( const sensor_msgs::Joy::ConstPtr &msg );
};

/*
 * JoyAxeInput : ROS Input for one Joystick's axe value
 * button : the ID of the button in the buttons's array
 */
class JoyButtonInput : public FScalar, public RosSubscriber<sensor_msgs::Joy>
{
        private :

                IString topic_name;
                ISInput size_queue;
                ISInput sleep;
                ISInput button;

        public :
                JoyButtonInput() : RosSubscriber<sensor_msgs::Joy>() {}
                virtual ~JoyButtonInput(){}

                virtual void uprerun();
                virtual void compute();
                virtual void setparameters();

                virtual void callback( const sensor_msgs::Joy::ConstPtr &msg );
};

#endif // __ROS_INPUT_HPP__
