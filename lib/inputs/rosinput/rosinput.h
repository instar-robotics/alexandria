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

#include <vector>
#include "kheops/kernel/function.h"
#include "kheops/kernel/kernel.h"
#include "kheops/ros/rossubscriber.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/LaserScan.h>


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

/*******************************************************************************************************/
/*****************                            Joystick Inputs                        *******************/
/*******************************************************************************************************/

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
		
		virtual void compute();
                virtual void setparameters();

		virtual void onQuit();
		virtual void onRun();
		virtual void onPause();
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
                JoyAxeInput() : FScalar() ,  RosSubscriber<sensor_msgs::Joy>() {}
                virtual ~JoyAxeInput(){}

                virtual void compute();
                virtual void setparameters();

		virtual void onQuit();
		virtual void onRun();
		virtual void onPause();
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
                JoyButtonsInput() : FMatrix(),  RosSubscriber<sensor_msgs::Joy>() {}
                virtual ~JoyButtonsInput(){}

                virtual void compute();
                virtual void setparameters();

		virtual void onQuit();
		virtual void onRun();
		virtual void onPause();
                virtual void callback( const sensor_msgs::Joy::ConstPtr &msg );
};

/*
 * JoyButtonInput : ROS Input for one Joystick's axe value
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

                virtual void compute();
                virtual void setparameters();

		virtual void onQuit();
		virtual void onRun();
		virtual void onPause();
                virtual void callback( const sensor_msgs::Joy::ConstPtr &msg );
};

/*******************************************************************************************************/
/*****************                          Odometry Inputs                          *******************/
/*******************************************************************************************************/

/*
 * OdoPosInput : ROS Input Odometric Cartesian Position
 * Vector with X,Y,Z absolute coordinate
 */
class OdoPosInput :  public FMatrix, public RosSubscriber<nav_msgs::Odometry>
{
	private : 

		IString topic_name;
                ISInput size_queue;
                ISInput sleep;

	public : 

		OdoPosInput() : RosSubscriber<nav_msgs::Odometry>() {}
		virtual ~OdoPosInput(){}

		virtual void uprerun();
                virtual void compute();
                virtual void setparameters();

		virtual void onQuit();
		virtual void onRun();
		virtual void onPause();
                virtual void callback( const nav_msgs::Odometry::ConstPtr &msg );
};

/*
 * OdoPosXInput : ROS Input Odometric Cartesian Position, X component
 * Point X
 */
class OdoPosXInput :  public FScalar, public RosSubscriber<nav_msgs::Odometry>
{
	private : 

                IString topic_name;
                ISInput size_queue;
                ISInput sleep;

        public : 

                OdoPosXInput() : RosSubscriber<nav_msgs::Odometry>(){}
                virtual ~OdoPosXInput(){}

                virtual void compute();
                virtual void setparameters();

		virtual void onQuit();
		virtual void onRun();
		virtual void onPause();
                virtual void callback( const nav_msgs::Odometry::ConstPtr &msg );
};

/*
 * OdoPosYInput : ROS Input Odometric Cartesian Position, Y component
 * Point Y
 */
class OdoPosYInput : public FScalar, public RosSubscriber<nav_msgs::Odometry>
{
	private :

                IString topic_name;
                ISInput size_queue;
                ISInput sleep;

        public :

                OdoPosYInput() : RosSubscriber<nav_msgs::Odometry>(){}
                virtual ~OdoPosYInput(){}

                virtual void compute();
                virtual void setparameters();

		virtual void onQuit();
		virtual void onRun();
		virtual void onPause();
                virtual void callback( const nav_msgs::Odometry::ConstPtr &msg );
};

/*
 * OdoPosZInput : ROS Input Odometric Cartesian Position, Z component
 * Point Z
 */
class OdoPosZInput : public FScalar, public RosSubscriber<nav_msgs::Odometry>
{
	private :

                IString topic_name;
                ISInput size_queue;
                ISInput sleep;

        public :

                OdoPosZInput() : RosSubscriber<nav_msgs::Odometry>(){}
                virtual ~OdoPosZInput(){}

                virtual void compute();
                virtual void setparameters();

		virtual void onQuit();
		virtual void onRun();
		virtual void onPause();
                virtual void callback( const nav_msgs::Odometry::ConstPtr &msg );
};

/*
 * OdoEulerInput : ROS Input Odometric Absolute Orientation in Euler coordinate (Roll, Pitch, Yaw)
 * Vector Roll, Pitch, Yaw
 */
class OdoEulerInput : public FMatrix, public RosSubscriber<nav_msgs::Odometry>
{
	private :

                IString topic_name;
                ISInput size_queue;
                ISInput sleep;
        
        public :

                OdoEulerInput() : RosSubscriber<nav_msgs::Odometry>(){}
                virtual ~OdoEulerInput(){}

		virtual void uprerun();
                virtual void compute();
                virtual void setparameters();

		virtual void onQuit();
		virtual void onRun();
		virtual void onPause();
                virtual void callback( const nav_msgs::Odometry::ConstPtr &msg );
};

/*
 * OdoEulerRollInput : ROS Input Odometric Absolute Orientation in Euler coordinate (Roll)
 * Scalar Roll
 */
class OdoEulerRollInput : public FScalar, public RosSubscriber<nav_msgs::Odometry>
{
	private :

                IString topic_name;
                ISInput size_queue;
                ISInput sleep;

        public :

                OdoEulerRollInput() : RosSubscriber<nav_msgs::Odometry>(){}
                virtual ~OdoEulerRollInput(){}

                virtual void compute();
                virtual void setparameters();

		virtual void onQuit();
		virtual void onRun();
		virtual void onPause();
                virtual void callback( const nav_msgs::Odometry::ConstPtr &msg );
}; 

/*
 * OdoEulerPitchInput : ROS Input Odometric Absolute Orientation in Euler coordinate (Pitch)
 * Scalar Pitch
 */
class OdoEulerPitchInput : public FScalar, public RosSubscriber<nav_msgs::Odometry>
{
	private :

                IString topic_name;
                ISInput size_queue;
                ISInput sleep;

        public :

                OdoEulerPitchInput() : RosSubscriber<nav_msgs::Odometry>(){}
                virtual ~OdoEulerPitchInput(){}

                virtual void compute();
                virtual void setparameters();

		virtual void onQuit();
		virtual void onRun();
		virtual void onPause();
                virtual void callback( const nav_msgs::Odometry::ConstPtr &msg );
};

/*
 * OdoEulerYawInput : ROS Input Odometric Absolute Orientation in Euler coordinate (Yaw)
 * Scalar Yaw
 */
class OdoEulerYawInput : public FScalar, public RosSubscriber<nav_msgs::Odometry>
{
	private :

                IString topic_name;
                ISInput size_queue;
                ISInput sleep;
        
        public :

                OdoEulerYawInput() : RosSubscriber<nav_msgs::Odometry>(){}
                virtual ~OdoEulerYawInput(){}

                virtual void compute();
                virtual void setparameters();

		virtual void onQuit();
		virtual void onRun();
		virtual void onPause();
                virtual void callback( const nav_msgs::Odometry::ConstPtr &msg );
};

/*
 * OdoQuaterInput : ROS Input Odometric Absolute Orientation in Quaternion coordinate (X,Y,Z,W)
 * Vector X,Y,Z,W
 */
class OdoQuaterInput : public FMatrix, public RosSubscriber<nav_msgs::Odometry>
{
	private :
                IString topic_name;
                ISInput size_queue;
                ISInput sleep;
        
        public :

                OdoQuaterInput() : RosSubscriber<nav_msgs::Odometry>(){}
                virtual ~OdoQuaterInput(){}

		virtual void uprerun();
                virtual void compute();
                virtual void setparameters();

		virtual void onQuit();
		virtual void onRun();
		virtual void onPause();
                virtual void callback( const nav_msgs::Odometry::ConstPtr &msg );
};

/*
 * OdoQuaterXInput : ROS Input Odometric Absolute Orientation in Quaternion coodinate (X)
 * Scalar X
 */
class OdoQuaterXInput : public FScalar, public RosSubscriber<nav_msgs::Odometry>
{
	private :

                IString topic_name;
                ISInput size_queue;
                ISInput sleep;

        public :

                OdoQuaterXInput() : RosSubscriber<nav_msgs::Odometry>(){}
                virtual ~OdoQuaterXInput(){}

                virtual void compute();
                virtual void setparameters();

		virtual void onQuit();
		virtual void onRun();
		virtual void onPause();
                virtual void callback( const nav_msgs::Odometry::ConstPtr &msg );
};

/*
 * OdoQuaterYInput : ROS Input Odometric Absolute Orientation in Quaternion coodinate (Y)
 * Scalar Y
 */
class OdoQuaterYInput : public FScalar, public RosSubscriber<nav_msgs::Odometry>
{
	private :

                IString topic_name;
                ISInput size_queue;
                ISInput sleep;
        
        public :

                OdoQuaterYInput() : RosSubscriber<nav_msgs::Odometry>(){}
                virtual ~OdoQuaterYInput(){}

                virtual void compute();
                virtual void setparameters();

		virtual void onQuit();
		virtual void onRun();
		virtual void onPause();
                virtual void callback( const nav_msgs::Odometry::ConstPtr &msg );
};


/*
 * OdoQuaterZInput : ROS Input Odometric Absolute Orientation in Quaternion coodinate (Z)
 * Scalar Z
 */
class OdoQuaterZInput : public FScalar, public RosSubscriber<nav_msgs::Odometry>
{
	private :

                IString topic_name;
                ISInput size_queue;
                ISInput sleep;
        
        public :

                OdoQuaterZInput() : RosSubscriber<nav_msgs::Odometry>(){}
                virtual ~OdoQuaterZInput(){}

                virtual void compute();
                virtual void setparameters();

		virtual void onQuit();
		virtual void onRun();
		virtual void onPause();
                virtual void callback( const nav_msgs::Odometry::ConstPtr &msg );
};


/*
 * OdoQuaterWInput : ROS Input Odometric Absolute Orientation in Quaternion coodinate (W)
 * Scalar W
 */
class OdoQuaterWInput : public FScalar, public RosSubscriber<nav_msgs::Odometry>
{
	private :

                IString topic_name;
                ISInput size_queue;
                ISInput sleep;
        
        public :

                OdoQuaterWInput() : RosSubscriber<nav_msgs::Odometry>(){}
                virtual ~OdoQuaterWInput(){}

                virtual void compute();
                virtual void setparameters();

		virtual void onQuit();
		virtual void onRun();
		virtual void onPause();
                virtual void callback( const nav_msgs::Odometry::ConstPtr &msg );
};


/*
 * OdoTwistLinInput : ROS Input Odometric Twist, Linear movement on (X,Y,Z)
 * Vector (X,Y,Z)
 */
class OdoTwistLinInput : public FMatrix, public RosSubscriber<nav_msgs::Odometry>
{
	private :

                IString topic_name;
                ISInput size_queue;
                ISInput sleep;
        
        public :

                OdoTwistLinInput() : RosSubscriber<nav_msgs::Odometry>(){}
                virtual ~OdoTwistLinInput(){}

		virtual void uprerun();
                virtual void compute();
                virtual void setparameters();

		virtual void onQuit();
		virtual void onRun();
		virtual void onPause();
                virtual void callback( const nav_msgs::Odometry::ConstPtr &msg );
};


/*
 * OdoTwistLinXInput : ROS Input Odometric Twist, Linear movement on X
 * Scalar X
 */
class OdoTwistLinXInput : public FScalar, public RosSubscriber<nav_msgs::Odometry>
{
	private :

                IString topic_name;
                ISInput size_queue;
                ISInput sleep;

        public :

                OdoTwistLinXInput() : RosSubscriber<nav_msgs::Odometry>(){}
                virtual ~OdoTwistLinXInput(){}

                virtual void compute();
                virtual void setparameters();

		virtual void onQuit();
		virtual void onRun();
		virtual void onPause();
                virtual void callback( const nav_msgs::Odometry::ConstPtr &msg );
}; 

/*
 * OdoTwistLinXInput : ROS Input Odometric Twist, Linear movement on Y
 * Scalar Y
 */
class OdoTwistLinYInput : public FScalar, public RosSubscriber<nav_msgs::Odometry>
{
	private :

                IString topic_name;
                ISInput size_queue;
                ISInput sleep;

        public :

                OdoTwistLinYInput() : RosSubscriber<nav_msgs::Odometry>(){}
                virtual ~OdoTwistLinYInput(){}

                virtual void compute();
                virtual void setparameters();

		virtual void onQuit();
		virtual void onRun();
		virtual void onPause();
                virtual void callback( const nav_msgs::Odometry::ConstPtr &msg );

};

/*
 * OdoTwistLinZInput : ROS Input Odometric Twist, Linear movement on Z
 * Scalar Z
 */
class OdoTwistLinZInput : public FScalar, public RosSubscriber<nav_msgs::Odometry>
{
	private :

                IString topic_name;
                ISInput size_queue;
                ISInput sleep;
        
        public :

                OdoTwistLinZInput() : RosSubscriber<nav_msgs::Odometry>(){}
                virtual ~OdoTwistLinZInput(){}

                virtual void compute();
                virtual void setparameters();

		virtual void onQuit();
		virtual void onRun();
		virtual void onPause();
                virtual void callback( const nav_msgs::Odometry::ConstPtr &msg );
};

/*
 * OdoTwistAngInput : ROS Input Odometric Twist, Angular movement on (Roll,Pitch,Yaw)
 * Vector (Roll, Pitch, Yaw)
 */
class OdoTwistAngInput : public FMatrix, public RosSubscriber<nav_msgs::Odometry>
{
	private :

                IString topic_name;
                ISInput size_queue;
                ISInput sleep;

        public :

                OdoTwistAngInput() : RosSubscriber<nav_msgs::Odometry>(){}
                virtual ~OdoTwistAngInput(){}

		virtual void uprerun();
                virtual void compute();
                virtual void setparameters();

		virtual void onQuit();
		virtual void onRun();
		virtual void onPause();
                virtual void callback( const nav_msgs::Odometry::ConstPtr &msg );

};

/*
 * OdoTwistAngRollInput : ROS Input Odometric Twist, Angular movement on (Roll)
 * Scalar (Roll)
 */
class OdoTwistAngRollInput : public FScalar, public RosSubscriber<nav_msgs::Odometry>
{
        private :

                IString topic_name;
                ISInput size_queue;
                ISInput sleep;

        public :

                OdoTwistAngRollInput() : RosSubscriber<nav_msgs::Odometry>(){}
                virtual ~OdoTwistAngRollInput(){}

                virtual void compute();
                virtual void setparameters();

		virtual void onQuit();
		virtual void onRun();
		virtual void onPause();
                virtual void callback( const nav_msgs::Odometry::ConstPtr &msg );
};

/*
 * OdoTwistAngPitchInput : ROS Input Odometric Twist, Angular movement on (Pitch)
 * Scalar (Pitch)
 */
class OdoTwistAngPitchInput : public FScalar, public RosSubscriber<nav_msgs::Odometry> 
{
        private :

                IString topic_name;
                ISInput size_queue;
                ISInput sleep;

        public :

                OdoTwistAngPitchInput() : RosSubscriber<nav_msgs::Odometry>(){}
                virtual ~OdoTwistAngPitchInput(){}

                virtual void compute();
                virtual void setparameters();

		virtual void onQuit();
		virtual void onRun();
		virtual void onPause();
                virtual void callback( const nav_msgs::Odometry::ConstPtr &msg );
};

/*
 * OdoTwistAngYawInput : ROS Input Odometric Twist, Angular movement on (Yaw)
 * Scalar (Yaw)
 */
class OdoTwistAngYawInput : public FScalar, public RosSubscriber<nav_msgs::Odometry> 
{
        private :

                IString topic_name;
                ISInput size_queue;
                ISInput sleep;

        public :

                OdoTwistAngYawInput() : RosSubscriber<nav_msgs::Odometry>(){}
                virtual ~OdoTwistAngYawInput(){}

                virtual void compute();
                virtual void setparameters();

		virtual void onQuit();
		virtual void onRun();
		virtual void onPause();
                virtual void callback( const nav_msgs::Odometry::ConstPtr &msg );
};

/*******************************************************************************************************/
/*****************                         LaserScan Inputs                          *******************/
/*******************************************************************************************************/

class LaserScanInput : public FMatrix, public RosSubscriber<sensor_msgs::LaserScan>
{
	private : 

		IString topic_name;
                ISInput size_queue;
                ISInput sleep;

                ISInput range_max;

		std::vector<unsigned int> moy; 

	public : 

                LaserScanInput() : RosSubscriber<sensor_msgs::LaserScan>(){}
                virtual ~LaserScanInput(){}

                virtual void compute();
                virtual void setparameters();

                virtual void onQuit();
                virtual void onRun();
                virtual void onPause();
                virtual void callback( const sensor_msgs::LaserScan::ConstPtr &msg );
};

#endif // __ROS_INPUT_HPP__
