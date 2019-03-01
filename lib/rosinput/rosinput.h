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
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h> 


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
 * JoyAxes : ROS Input for Joystick's axes values
 * Axes array must have the same dimension than the Output Matrix
 * But Matrix could be in any row/col form
 */
class JoyAxes : public FMatrix, public RosSubscriber<sensor_msgs::Joy>
{
	private :

		IString topic_name;
		ISInput size_queue;
		ISInput sleep;

	public : 
		JoyAxes() : RosSubscriber<sensor_msgs::Joy>() {}
		virtual ~JoyAxes(){}
		
		virtual void compute();
                virtual void setparameters();

		virtual void onQuit();
		virtual void onRun();
		virtual void onPause();
		virtual void callback( const sensor_msgs::Joy::ConstPtr &msg );
};

/*
 * JoyAxe : ROS Input for one Joystick's axe value
 * axe : the ID of the axe in the axes's array
 */
class JoyAxe : public FScalar, public RosSubscriber<sensor_msgs::Joy>
{
        private :

                IString topic_name;
                ISInput size_queue;
                ISInput sleep;
                ISInput axe;

        public :
                JoyAxe() : FScalar() ,  RosSubscriber<sensor_msgs::Joy>() {}
                virtual ~JoyAxe(){}

                virtual void compute();
                virtual void setparameters();

		virtual void onQuit();
		virtual void onRun();
		virtual void onPause();
                virtual void callback( const sensor_msgs::Joy::ConstPtr &msg );
};

/*
 * JoyButtons : ROS Input for Joystick's buttons values
 * Buttons array must have the same dimension than the Output Matrix
 * But Matrix could be in any row/col form
 */
class JoyButtons : public FMatrix, public RosSubscriber<sensor_msgs::Joy>
{
        private :

                IString topic_name;
                ISInput size_queue;
                ISInput sleep;

        public :
                JoyButtons() : FMatrix(),  RosSubscriber<sensor_msgs::Joy>() {}
                virtual ~JoyButtons(){}

                virtual void compute();
                virtual void setparameters();

		virtual void onQuit();
		virtual void onRun();
		virtual void onPause();
                virtual void callback( const sensor_msgs::Joy::ConstPtr &msg );
};

/*
 * JoyButton : ROS Input for one Joystick's axe value
 * button : the ID of the button in the buttons's array
 */
class JoyButton : public FScalar, public RosSubscriber<sensor_msgs::Joy>
{
        private :

                IString topic_name;
                ISInput size_queue;
                ISInput sleep;
                ISInput button;

        public :
                JoyButton() : RosSubscriber<sensor_msgs::Joy>() {}
                virtual ~JoyButton(){}

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

/*******************************************************************************************************/
/*****************                          Lidar1D Inputs                           *******************/
/*******************************************************************************************************/

class Lidar1D : public FMatrix, public RosSubscriber<sensor_msgs::LaserScan>
{
	private : 

		IString topic_name;
                ISInput size_queue;
                ISInput sleep;

                ISInput range_max;

		std::vector<unsigned int> moy; 

	public : 

                Lidar1D() : FMatrix(VECTOR) , RosSubscriber<sensor_msgs::LaserScan>(){}
                virtual ~Lidar1D(){}

                virtual void compute();
                virtual void setparameters();

                virtual void onQuit();
                virtual void onRun();
                virtual void onPause();
                virtual void callback( const sensor_msgs::LaserScan::ConstPtr &msg );
};

/*******************************************************************************************************/
/*****************                          Lidar2D Inputs                           *******************/
/*******************************************************************************************************/

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;


//class Lidar2D : public FMatrix, public RosSubscriber<sensor_msgs::PointCloud2>
class Lidar2D : public FMatrix, public RosSubscriber<PointCloud>
{
        private :

                IString topic_name;
                ISInput size_queue;
                ISInput sleep;

                ISInput range_max;

		MatrixXi moy; 

        public :

                //Lidar2D() : RosSubscriber<sensor_msgs::PointCloud2>(){}
                Lidar2D() : RosSubscriber<PointCloud>(){}
                virtual ~Lidar2D(){}

                virtual void compute();
                virtual void setparameters();

                virtual void onQuit();
                virtual void onRun();
                virtual void onPause();

                //virtual void callback( const sensor_msgs::PointCloud2::ConstPtr &msg );
                virtual void callback( const PointCloud::ConstPtr& msg );
};

/*******************************************************************************************************/
/*****************                     		    3D Compass	      	                   *******************/
/*******************************************************************************************************/


class Compass3D : public FMatrix, public RosSubscriber<sensor_msgs::Imu>
{
	private :

		IString topic_name;
		ISInput size_queue;
		ISInput sleep;

	public : 
		Compass3D() : RosSubscriber<sensor_msgs::Imu>() {}
		virtual ~Compass3D(){}
		
		virtual void prerun();
		virtual void compute();
		virtual void setparameters();

		virtual void onQuit();
		virtual void onRun();
		virtual void onPause();
		virtual void callback( const sensor_msgs::Imu::ConstPtr &msg );

};

/*******************************************************************************************************/
/*****************                     		    3D Compass X      	                   *******************/
/*******************************************************************************************************/


class CompassX : public FScalar, public RosSubscriber<sensor_msgs::Imu>
{
	private :

		IString topic_name;
		ISInput size_queue;
		ISInput sleep;

	public : 
		CompassX() : RosSubscriber<sensor_msgs::Imu>() {}
		virtual ~CompassX(){}
		
		virtual void compute();
		virtual void setparameters();
		virtual void onQuit();
		virtual void onRun();
		virtual void onPause();
		virtual void callback( const sensor_msgs::Imu::ConstPtr &msg );

};

/*******************************************************************************************************/
/*****************                     		    3D Compass Y      	                   *******************/
/*******************************************************************************************************/


class CompassY : public FScalar, public RosSubscriber<sensor_msgs::Imu>
{
	private :

		IString topic_name;
		ISInput size_queue;
		ISInput sleep;

	public : 
		CompassY() : RosSubscriber<sensor_msgs::Imu>() {}
		virtual ~CompassY(){}
		
		virtual void compute();
		virtual void setparameters();
		virtual void onQuit();
		virtual void onRun();
		virtual void onPause();
		virtual void callback( const sensor_msgs::Imu::ConstPtr &msg );

};

/*******************************************************************************************************/
/*****************                     		    3D Compass Z      	                   *******************/
/*******************************************************************************************************/


class CompassZ : public FScalar, public RosSubscriber<sensor_msgs::Imu>
{
	private :

		IString topic_name;
		ISInput size_queue;
		ISInput sleep;

	public : 
		CompassZ() : RosSubscriber<sensor_msgs::Imu>() {}
		virtual ~CompassZ(){}
		
		virtual void compute();
		virtual void setparameters();
		virtual void onQuit();
		virtual void onRun();
		virtual void onPause();
		virtual void callback( const sensor_msgs::Imu::ConstPtr &msg );

};

/*******************************************************************************************************/
/*****************                     	    3D Gyroscope	      	                   *******************/
/*******************************************************************************************************/


class Gyroscope3D : public FMatrix, public RosSubscriber<sensor_msgs::Imu>
{
	private :

		IString topic_name;
		ISInput size_queue;
		ISInput sleep;

	public : 
		Gyroscope3D() : RosSubscriber<sensor_msgs::Imu>() {}
		virtual ~Gyroscope3D(){}
		
		virtual void prerun();
		virtual void compute();
		virtual void setparameters();

		virtual void onQuit();
		virtual void onRun();
		virtual void onPause();
		virtual void callback( const sensor_msgs::Imu::ConstPtr &msg );

};

/*******************************************************************************************************/
/*****************                     	    Gyroscope X 	      	                   *******************/
/*******************************************************************************************************/


class GyroscopeX : public FScalar, public RosSubscriber<sensor_msgs::Imu>
{
	private :

		IString topic_name;
		ISInput size_queue;
		ISInput sleep;

	public : 
		GyroscopeX() : RosSubscriber<sensor_msgs::Imu>() {}
		virtual ~GyroscopeX(){}
		
		virtual void compute();
		virtual void setparameters();

		virtual void onQuit();
		virtual void onRun();
		virtual void onPause();
		virtual void callback( const sensor_msgs::Imu::ConstPtr &msg );

};

/*******************************************************************************************************/
/*****************                     	    Gyroscope Y	      	                   *******************/
/*******************************************************************************************************/


class GyroscopeY : public FScalar, public RosSubscriber<sensor_msgs::Imu>
{
	private :

		IString topic_name;
		ISInput size_queue;
		ISInput sleep;

	public : 
		GyroscopeY() : RosSubscriber<sensor_msgs::Imu>() {}
		virtual ~GyroscopeY(){}
		
		virtual void compute();
		virtual void setparameters();

		virtual void onQuit();
		virtual void onRun();
		virtual void onPause();
		virtual void callback( const sensor_msgs::Imu::ConstPtr &msg );

};

/*******************************************************************************************************/
/*****************                     	    Gyroscope Z 	      	                   *******************/
/*******************************************************************************************************/


class GyroscopeZ : public FScalar, public RosSubscriber<sensor_msgs::Imu>
{
	private :

		IString topic_name;
		ISInput size_queue;
		ISInput sleep;

	public : 
		GyroscopeZ() : RosSubscriber<sensor_msgs::Imu>() {}
		virtual ~GyroscopeZ(){}
		
		virtual void compute();
		virtual void setparameters();

		virtual void onQuit();
		virtual void onRun();
		virtual void onPause();
		virtual void callback( const sensor_msgs::Imu::ConstPtr &msg );

};

/*******************************************************************************************************/
/*****************                   	    3D Accelerometer	     	                   *******************/
/*******************************************************************************************************/


class Accelerometer3D : public FMatrix, public RosSubscriber<sensor_msgs::Imu>
{
	private :

		IString topic_name;
		ISInput size_queue;
		ISInput sleep;

	public : 
		Accelerometer3D() : RosSubscriber<sensor_msgs::Imu>() {}
		virtual ~Accelerometer3D(){}
		
		virtual void prerun();
		virtual void compute();
		virtual void setparameters();

		virtual void onQuit();
		virtual void onRun();
		virtual void onPause();
		virtual void callback( const sensor_msgs::Imu::ConstPtr &msg );

};

/*******************************************************************************************************/
/*****************                   	    Accelerometer X 	     	                   *******************/
/*******************************************************************************************************/


class AccelerometerX : public FScalar, public RosSubscriber<sensor_msgs::Imu>
{
	private :

		IString topic_name;
		ISInput size_queue;
		ISInput sleep;

	public : 
		AccelerometerX() : RosSubscriber<sensor_msgs::Imu>() {}
		virtual ~AccelerometerX(){}
		
		virtual void compute();
		virtual void setparameters();

		virtual void onQuit();
		virtual void onRun();
		virtual void onPause();
		virtual void callback( const sensor_msgs::Imu::ConstPtr &msg );

};

/*******************************************************************************************************/
/*****************                   	    Accelerometer Y 	     	                   *******************/
/*******************************************************************************************************/


class AccelerometerY : public FScalar, public RosSubscriber<sensor_msgs::Imu>
{
	private :

		IString topic_name;
		ISInput size_queue;
		ISInput sleep;

	public : 
		AccelerometerY() : RosSubscriber<sensor_msgs::Imu>() {}
		virtual ~AccelerometerY(){}
		
		virtual void compute();
		virtual void setparameters();

		virtual void onQuit();
		virtual void onRun();
		virtual void onPause();
		virtual void callback( const sensor_msgs::Imu::ConstPtr &msg );

};

/*******************************************************************************************************/
/*****************                   	    Accelerometer Z 	     	                   *******************/
/*******************************************************************************************************/


class AccelerometerZ : public FScalar, public RosSubscriber<sensor_msgs::Imu>
{
	private :

		IString topic_name;
		ISInput size_queue;
		ISInput sleep;

	public : 
		AccelerometerZ() : RosSubscriber<sensor_msgs::Imu>() {}
		virtual ~AccelerometerZ(){}
		
		virtual void compute();
		virtual void setparameters();

		virtual void onQuit();
		virtual void onRun();
		virtual void onPause();
		virtual void callback( const sensor_msgs::Imu::ConstPtr &msg );

};


#endif // __ROS_INPUT_HPP__
