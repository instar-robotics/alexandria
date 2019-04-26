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

#ifndef __SENSOR_HPP__
#define __SENSOR_HPP__

#include "kheops/kernel/function.h"
#include "kheops/kernel/kernel.h"
#include "kheops/ros/rossubscriber.h"
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <vector>

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
/*****************                       LaserScan Inputs                            *******************/
/*******************************************************************************************************/

class LaserScan : public FMatrix, public RosSubscriber<sensor_msgs::LaserScan>
{
        private :

                IString topic_name;
                ISInput size_queue;
                ISInput sleep;

                ISInput range_max;

                std::vector<unsigned int> moy;

        public :

                LaserScan() : FMatrix(VECTOR) , RosSubscriber<sensor_msgs::LaserScan>(){}
                virtual ~LaserScan(){}

                virtual void compute();
                virtual void setparameters();

                virtual void onQuit();
                virtual void onRun();
                virtual void onPause();
                virtual void callback( const sensor_msgs::LaserScan::ConstPtr &msg );
};

/*******************************************************************************************************/
/*****************                           3D Compass                              *******************/
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
/*****************                            3D Compass X                           *******************/
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
/*****************                            3D Compass Y                           *******************/
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
/*****************                            3D Compass Z                           *******************/
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
/*****************                          3D Gyroscope                             *******************/
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
/*****************                          Gyroscope X                              *******************/
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
/*****************                           Gyroscope Y                             *******************/
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
/*****************                          Gyroscope Z                              *******************/
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
/*****************                        3D Accelerometer                           *******************/
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
/*****************                         Accelerometer X                           *******************/
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
/*****************                        Accelerometer Y                            *******************/
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
/***************                          Accelerometer Z                            *******************/
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

#endif // __SENSOR_HPP__
