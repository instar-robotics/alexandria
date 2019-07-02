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

#ifndef __SENSOR_MSGS_HPP__
#define __SENSOR_MSGS_HPP__

#include "kheops/ros/fsub.h"
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <vector>

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
/*****************                            Joystick Sub                           *******************/
/*******************************************************************************************************/

/*
 * JoyAxesSub : ROS Subscriber for Joystick's axes values
 * Axes array must have the same dimension than the Output Vector
 */
class JoyAxesSub : public FMatrixSub<sensor_msgs::Joy>
{
        public :
                JoyAxesSub() : FMatrixSub<sensor_msgs::Joy>(VECTOR) {}
                virtual ~JoyAxesSub(){}

                virtual void callback( const sensor_msgs::Joy::ConstPtr &msg );
};


/*
 * JoyAxeSub : ROS Subscriber for one Joystick's axe value
 * axe : the ID of the axe in the axes's array
 */
class JoyAxeSub : public FScalarSub<sensor_msgs::Joy>
{
	private :

                ISInput axe;

        public :
                JoyAxeSub() : FScalarSub<sensor_msgs::Joy>() {}
                virtual ~JoyAxeSub(){}

                virtual void setparameters();
                virtual void callback( const sensor_msgs::Joy::ConstPtr &msg );
};


/*
 * JoyButtonsSub : ROS Subscriber for Joystick's buttons values
 * Buttons array must have the same dimension than the Output Vector
 */
class JoyButtonsSub : public FMatrixSub<sensor_msgs::Joy>
{
        public :
                JoyButtonsSub() : FMatrixSub<sensor_msgs::Joy>(VECTOR) {}
                virtual ~JoyButtonsSub(){}

                virtual void callback( const sensor_msgs::Joy::ConstPtr &msg );
};


/*
 * JoyButtonSub : ROS Subscriber for one Joystick's axe value
 * button : the ID of the button in the buttons's array
 */
class JoyButtonSub : public FScalarSub<sensor_msgs::Joy>
{
	private :

                ISInput button;

        public :
                JoyButtonSub() : FScalarSub<sensor_msgs::Joy>() {}
                virtual ~JoyButtonSub(){}

                virtual void setparameters();
                virtual void callback( const sensor_msgs::Joy::ConstPtr &msg );
};

/*******************************************************************************************************/
/*****************                          LaserScanSub                             *******************/
/*******************************************************************************************************/

class LaserScanSub : public FMatrixSub<sensor_msgs::LaserScan>
{
        private :

                ISInput range_max;

                std::vector<unsigned int> moy;

        public :

                LaserScanSub() : FMatrixSub<sensor_msgs::LaserScan>(VECTOR){}
                virtual ~LaserScanSub(){}

                virtual void setparameters();
                virtual void callback( const sensor_msgs::LaserScan::ConstPtr &msg );
};

/*******************************************************************************************************/
/*******************                        PointCloud2Sub                           *******************/
/*******************************************************************************************************/

class PointCloud2Sub : public FMatrixSub<sensor_msgs::PointCloud2>
{
        private :

                ISInput range_max;
                MatrixXi moy;

        public :

                PointCloud2Sub() : FMatrixSub<sensor_msgs::PointCloud2>(){}
                virtual ~PointCloud2Sub(){}

                virtual void setparameters();
                virtual void callback( const sensor_msgs::PointCloud2::ConstPtr &msg );
};

/*******************************************************************************************************/
/*****************                           3D Compass                              *******************/
/*******************************************************************************************************/

class Compass3DSub : public FMatrixSub<sensor_msgs::Imu>
{
        public :
                Compass3DSub() : FMatrixSub<sensor_msgs::Imu>(VECTOR) {}
                virtual ~Compass3DSub(){}

                virtual void setparameters();
                virtual void callback( const sensor_msgs::Imu::ConstPtr &msg );
};

/*******************************************************************************************************/
/*****************                            3D Compass X                           *******************/
/*******************************************************************************************************/

class CompassXSub : public FScalarSub<sensor_msgs::Imu>
{
        public :
                CompassXSub() : FScalarSub<sensor_msgs::Imu>() {}
                virtual ~CompassXSub(){}

                virtual void callback( const sensor_msgs::Imu::ConstPtr &msg );
};

/*******************************************************************************************************/
/*****************                            3D Compass Y                           *******************/
/*******************************************************************************************************/

class CompassYSub : public FScalarSub<sensor_msgs::Imu>
{
        public :
                CompassYSub() : FScalarSub<sensor_msgs::Imu>() {}
                virtual ~CompassYSub(){}

                virtual void callback( const sensor_msgs::Imu::ConstPtr &msg );
};

/*******************************************************************************************************/
/*****************                            3D Compass Z                           *******************/
/*******************************************************************************************************/

class CompassZSub : public FScalarSub<sensor_msgs::Imu>
{
        public :
                CompassZSub() : FScalarSub<sensor_msgs::Imu>() {}
                virtual ~CompassZSub(){}

                virtual void callback( const sensor_msgs::Imu::ConstPtr &msg );
};

/*******************************************************************************************************/
/*****************                          3D Gyroscope                             *******************/
/*******************************************************************************************************/

class Gyroscope3DSub : public FMatrixSub<sensor_msgs::Imu>
{
        public :
                Gyroscope3DSub() : FMatrixSub<sensor_msgs::Imu>(VECTOR) {}
                virtual ~Gyroscope3DSub(){}

                virtual void setparameters();
                virtual void callback( const sensor_msgs::Imu::ConstPtr &msg );
};

/*******************************************************************************************************/
/*****************                           Gyroscope X                             *******************/
/*******************************************************************************************************/

class GyroscopeXSub : public FScalarSub<sensor_msgs::Imu>
{
        public :
                GyroscopeXSub() : FScalarSub<sensor_msgs::Imu>() {}
                virtual ~GyroscopeXSub(){}

                virtual void callback( const sensor_msgs::Imu::ConstPtr &msg );
};

/*******************************************************************************************************/
/*****************                            Gyroscope Y                            *******************/
/*******************************************************************************************************/

class GyroscopeYSub : public FScalarSub<sensor_msgs::Imu>
{
        public :
                GyroscopeYSub() : FScalarSub<sensor_msgs::Imu>() {}
                virtual ~GyroscopeYSub(){}

                virtual void callback( const sensor_msgs::Imu::ConstPtr &msg );
};

/*******************************************************************************************************/
/*****************                            Gyroscope Z                            *******************/
/*******************************************************************************************************/

class GyroscopeZSub : public FScalarSub<sensor_msgs::Imu>
{
        public :
                GyroscopeZSub() : FScalarSub<sensor_msgs::Imu>() {}
                virtual ~GyroscopeZSub(){}

                virtual void callback( const sensor_msgs::Imu::ConstPtr &msg );
};

/*******************************************************************************************************/
/*****************                        3D Accelerometer                           *******************/
/*******************************************************************************************************/

class Accelerometer3DSub : public FMatrixSub<sensor_msgs::Imu>
{
        public :
                Accelerometer3DSub() : FMatrixSub<sensor_msgs::Imu>(VECTOR) {}
                virtual ~Accelerometer3DSub(){}

                virtual void setparameters();
                virtual void callback( const sensor_msgs::Imu::ConstPtr &msg );
};

/*******************************************************************************************************/
/*****************                         Accelerometer X                           *******************/
/*******************************************************************************************************/

class AccelerometerXSub : public FScalarSub<sensor_msgs::Imu>
{
        public :
                AccelerometerXSub() : FScalarSub<sensor_msgs::Imu>() {}
                virtual ~AccelerometerXSub(){}

                virtual void callback( const sensor_msgs::Imu::ConstPtr &msg );
};

/*******************************************************************************************************/
/*****************                        Accelerometer Y                            *******************/
/*******************************************************************************************************/

class AccelerometerYSub : public FScalarSub<sensor_msgs::Imu>
{
        public :
                AccelerometerYSub() : FScalarSub<sensor_msgs::Imu>() {}
                virtual ~AccelerometerYSub(){}

                virtual void callback( const sensor_msgs::Imu::ConstPtr &msg );
};

/*******************************************************************************************************/
/****************                         Accelerometer Z                            *******************/
/*******************************************************************************************************/

class AccelerometerZSub : public FScalarSub<sensor_msgs::Imu>
{
        public :
                AccelerometerZSub() : FScalarSub<sensor_msgs::Imu>() {}
                virtual ~AccelerometerZSub(){}

                virtual void callback( const sensor_msgs::Imu::ConstPtr &msg );
};

/*******************************************************************************************************/
/****************                           NavSatFix                                *******************/
/*******************************************************************************************************/

class NavSatFixSub : public FMatrixSub<sensor_msgs::NavSatFix>
{
	public : 
		NavSatFixSub() : FMatrixSub<sensor_msgs::NavSatFix>(VECTOR) {}
		virtual ~NavSatFixSub() {}

		virtual void setparameters();
                virtual void callback( const sensor_msgs::NavSatFix::ConstPtr &msg );
};

/*******************************************************************************************************/
/****************                          NavSatFixLat                              *******************/
/*******************************************************************************************************/

class NavSatFixLatSub : public FScalarSub<sensor_msgs::NavSatFix>
{
	public : 
		NavSatFixLatSub() : FScalarSub<sensor_msgs::NavSatFix>() {}
		virtual ~NavSatFixLatSub() {}

                virtual void callback( const sensor_msgs::NavSatFix::ConstPtr &msg );
};

/*******************************************************************************************************/
/****************                         NavSatFixLong                              *******************/
/*******************************************************************************************************/

class NavSatFixLongSub : public FScalarSub<sensor_msgs::NavSatFix>
{
	public : 
		NavSatFixLongSub() : FScalarSub<sensor_msgs::NavSatFix>() {}
		virtual ~NavSatFixLongSub() {}

                virtual void callback( const sensor_msgs::NavSatFix::ConstPtr &msg );
};

/*******************************************************************************************************/
/****************                         NavSatFixAlt                              *******************/
/*******************************************************************************************************/

class NavSatFixAltSub : public FScalarSub<sensor_msgs::NavSatFix>
{
	public : 
		NavSatFixAltSub() : FScalarSub<sensor_msgs::NavSatFix>() {}
		virtual ~NavSatFixAltSub() {}

                virtual void callback( const sensor_msgs::NavSatFix::ConstPtr &msg );
};

#endif // __SENSOR_MSGS_HPP__
