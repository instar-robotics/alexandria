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

#include "sensors_sub.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

REGISTER_FUNCTION(JoyAxesSub);
REGISTER_FUNCTION(JoyAxeSub);
REGISTER_FUNCTION(JoyButtonsSub);
REGISTER_FUNCTION(JoyButtonSub);
REGISTER_FUNCTION(LaserScanSub);
REGISTER_FUNCTION(PointCloud2Sub);
REGISTER_FUNCTION(Compass3DSub);
REGISTER_FUNCTION(CompassXSub);
REGISTER_FUNCTION(CompassYSub);
REGISTER_FUNCTION(CompassZSub);
REGISTER_FUNCTION(Gyroscope3DSub);
REGISTER_FUNCTION(GyroscopeXSub);
REGISTER_FUNCTION(GyroscopeYSub);
REGISTER_FUNCTION(GyroscopeZSub);
REGISTER_FUNCTION(Accelerometer3DSub);
REGISTER_FUNCTION(AccelerometerXSub);
REGISTER_FUNCTION(AccelerometerYSub);
REGISTER_FUNCTION(AccelerometerZSub);
REGISTER_FUNCTION(NavSatFixSub);
REGISTER_FUNCTION(NavSatFixLatSub);
REGISTER_FUNCTION(NavSatFixLongSub);
REGISTER_FUNCTION(NavSatFixAltSub);

/*******************************************************************************************************/
/*****************                            JoyAxesSub                             *******************/
/*******************************************************************************************************/

void JoyAxesSub::callback( const sensor_msgs::Joy::ConstPtr &msg)
{
        if( msg->axes.size() != output.rows() * output.cols() )
        {
                throw std::invalid_argument("JoyAxesSub : Output dimension is not egal to the numbers of Joystick axes !");
        }

        auto mout = getMapVect(output);
        for(unsigned int i = 0; i < msg->axes.size() ; i++ )
        {
                mout[i] = msg->axes[i];
        }
}

/*******************************************************************************************************/
/******************                            JoyAxeSub                             *******************/
/*******************************************************************************************************/

void JoyAxeSub::setparameters()
{
	FScalarSub<sensor_msgs::Joy>::setparameters();
        Kernel::iBind(axe,"axe", getUuid());
}

void JoyAxeSub::callback( const sensor_msgs::Joy::ConstPtr &msg)
{
        if( axe()() >  msg->axes.size())
        {
                throw std::invalid_argument("JoyAxeSub : axe ID is out of range !");
        }

        output = msg->axes[ (int)(axe()()) - 1];
}

/*******************************************************************************************************/
/*****************                           JoyButtonsSub                           *******************/
/*******************************************************************************************************/

void JoyButtonsSub::callback( const sensor_msgs::Joy::ConstPtr &msg)
{
        if( msg->buttons.size() != output.rows() * output.cols() )
        {
                throw std::invalid_argument("JoyButtonsSub : Output dimension is not egal to the numbers of Joystick axes !");
        }

        auto mout = getMapVect(output);
        for(unsigned int i = 0; i < msg->buttons.size() ; i++ )
        {
                mout[i] = msg->buttons[i];
        }
}

/*******************************************************************************************************/
/*****************                            JoyButtonSub                           *******************/
/*******************************************************************************************************/

void JoyButtonSub::setparameters()
{
	FScalarSub<sensor_msgs::Joy>::setparameters();
        Kernel::iBind(button,"button", getUuid());
}

void JoyButtonSub::callback( const sensor_msgs::Joy::ConstPtr &msg)
{
        if( button()() >  msg->buttons.size())
        {
                throw std::invalid_argument("JoyButtonSub : button ID is out of range !");
        }

        output = msg->buttons[ (int)(button()()) - 1];
}

/*******************************************************************************************************/
/***********************                     LaserScanSub                       ************************/
/*******************************************************************************************************/

void LaserScanSub::setparameters()
{
	FMatrixSub<sensor_msgs::LaserScan>::setparameters();
        Kernel::iBind(range_max,"range_max", getUuid());
        
	moy.assign(output.size(), 0);
}

void LaserScanSub::callback(const sensor_msgs::LaserScan::ConstPtr &msg )
{
        auto mout = getMapVect(output);
        SCALAR RM = std::min(range_max()(),  (SCALAR)(msg->range_max) );
        SCALAR offset =  M_PI - fabs(msg->angle_min);

        for( unsigned int i = 0 ; i <  msg->ranges.size() ; i++)
        {
                unsigned int j = ( i * (msg->angle_max - msg->angle_min) /  msg->ranges.size() + offset ) * ( mout.size() / (2* M_PI)) ;

                SCALAR value = 1 - (msg->ranges[i] - msg->range_min) / (RM - msg->range_min) ;
                if( value < 0 ) value = 0;

		if( j < mout.size() ) 
		{
                        if( moy[j] == 0 ) mout(j) = value;
                        else mout(j) += value;

                        moy[j]++;
		}
        }

        for(unsigned int i = 0 ; i < mout.size() ; i++)
        {
                if( moy[i] > 0 ) mout(i) = mout(i) / moy[i];
                else mout(i) = 0;

                moy[i] = 0;
        }
}



/*******************************************************************************************************/
/************************                    PointCloud2Sub                    *************************/
/*******************************************************************************************************/

void PointCloud2Sub::setparameters()
{
	FMatrixSub<sensor_msgs::PointCloud2>::setparameters();
        Kernel::iBind(range_max,"range_max", getUuid());
        moy = MatrixXi::Constant(output.rows(), output.cols(),0);
}

void PointCloud2Sub::callback(const sensor_msgs::PointCloud2::ConstPtr &msg )
{
	/*
        output = MATRIX::Constant(output.rows(),output.cols(),0);

        for( unsigned int i = 0 ; i <  msg->points.size() ; i++ )
        {


                 SCALAR theta = (2 * atan( msg->points[i].y / ( msg->points[i].x + sqrt( msg->points[i].x * msg->points[i].x + msg->points[i].y * msg->points[i].y   ))) + M_PI ) *  output.cols() / (2*M_PI);

                 SCALAR value = sqrt( msg->points[i].x * msg->points[i].x + msg->points[i].y * msg->points[i].y + msg->points[i].z * msg->points[i].z  );

                 std::cout <<  msg->points[i].z+10  << " " << theta << " "<< value << std::endl;
                 if( ! std::isnan(theta) )  output( msg->points[i].z+10 , theta) = value;
        }
*/

        /*
        std::cout << msg->points.size() << std::endl;

        std::cout << msg->width << " " << msg->height << std::endl;

        auto const m = msg->getMatrixXfMap();
        std::cout << m.rows() << " . " << m.cols() << std::endl;

        for( unsigned int i = 0 ; i < m.cols() ; i++)
                for(unsigned int j = 0; j < m.rows(); j++)
        {
                output( j * output.rows() / m.rows() , i * output.cols() / m.cols() ) = m(j,i);
        }
*/


//      std::cout << m << std::endl;


        /*
        for( unsigned int i = 0 ; i <  msg->points.size() ; i++ )
        {
                 std::cout << msg->points[i].x << "  " << msg->points[i].y << " " <<  msg->points[i].z << std::endl;
        }
        */
        /*
        std::cout << "Width : " << msg->width << std::endl;
        std::cout << "Height : " << msg->height << std::endl;
        std::cout << "Point Step : " << msg->point_step << std::endl;
        std::cout << "Row Step : " << msg->row_step << std::endl;


        std::cout << "Point Fields Size : " << msg->fields.size() << std::endl;

        for(unsigned int i =0 ; i < msg->fields.size() ; i++)
        {
                std::cout << "\tPF Name : " << msg->fields[i].name << std::endl;
                std::cout << "\tPF offset : " << msg->fields[i].offset << std::endl;
                std::cout << "\tPF datatype : " << (int)msg->fields[i].datatype << std::endl;
                std::cout << "\tPF count : " << msg->fields[i].count << std::endl;

        }*/
}

/*******************************************************************************************************/
/*****************                   	     3D Compass                              *******************/
/*******************************************************************************************************/

void Compass3DSub::prerun()
{
	if( output.rows() * output.cols() != 3 ) throw std::invalid_argument("Compass3D : Output dimension should be 3 !");
}

void Compass3DSub::callback( const sensor_msgs::Imu::ConstPtr &msg)
{
	tf2::Quaternion q(msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);

	tf2::Matrix3x3 m(q);
	SCALAR roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);

	auto mout = getMapVect(output);
	mout[0] = roll;
	mout[1] = pitch;
	mout[2] = yaw;
}

/*******************************************************************************************************/
/*****************                             3D Compass X  		             *******************/
/*******************************************************************************************************/

void CompassXSub::callback( const sensor_msgs::Imu::ConstPtr &msg)
{
	tf2::Quaternion q(msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);

	tf2::Matrix3x3 m(q);
	SCALAR roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);

	output = roll;
}

/*******************************************************************************************************/
/*****************                             3D Compass Y                          *******************/
/*******************************************************************************************************/

void CompassYSub::callback( const sensor_msgs::Imu::ConstPtr &msg)
{
	tf2::Quaternion q(msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);

	tf2::Matrix3x3 m(q);
	SCALAR roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);

	output = pitch;
}

/*******************************************************************************************************/
/*****************                             3D Compass Z                          *******************/
/*******************************************************************************************************/

void CompassZSub::callback( const sensor_msgs::Imu::ConstPtr &msg)
{
	tf2::Quaternion q(msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);

	tf2::Matrix3x3 m(q);
	SCALAR roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);

	output = yaw;
}

/*******************************************************************************************************/
/*****************                            3D Gyroscope                           *******************/
/*******************************************************************************************************/

void Gyroscope3DSub::prerun()
{
	if( output.rows() * output.cols() != 3 ) throw std::invalid_argument("Gyroscope3DSub : Output dimension should be 3 !");
}

void Gyroscope3DSub::callback( const sensor_msgs::Imu::ConstPtr &msg)
{
	auto mout = getMapVect(output);
	mout[0] = msg->angular_velocity.x;
	mout[1] = msg->angular_velocity.y;
	mout[2] = msg->angular_velocity.z;
}

/*******************************************************************************************************/
/*****************                             Gyroscope X                           *******************/
/*******************************************************************************************************/

void GyroscopeXSub::callback( const sensor_msgs::Imu::ConstPtr &msg)
{
	output = msg->angular_velocity.x;
}

/*******************************************************************************************************/
/*****************                             Gyroscope Y                           *******************/
/*******************************************************************************************************/

void GyroscopeYSub::callback( const sensor_msgs::Imu::ConstPtr &msg)
{
	output = msg->angular_velocity.y;
}

/*******************************************************************************************************/
/*****************                             Gyroscope Z                           *******************/
/*******************************************************************************************************/

void GyroscopeZSub::callback( const sensor_msgs::Imu::ConstPtr &msg)
{
	output = msg->angular_velocity.z;
}

/*******************************************************************************************************/
/*****************                           3D Accelerometer                        *******************/
/*******************************************************************************************************/

void Accelerometer3DSub::prerun()
{
	if( output.rows() * output.cols() != 3 ) throw std::invalid_argument("Accelerometer3D : Output dimension should be 3 !");
}

void Accelerometer3DSub::callback( const sensor_msgs::Imu::ConstPtr &msg)
{
	auto mout = getMapVect(output);
	mout[0] = msg->linear_acceleration.x;
	mout[1] = msg->linear_acceleration.y;
	mout[2] = msg->linear_acceleration.z;
}

/*******************************************************************************************************/
/*****************                           Accelerometer X                         *******************/
/*******************************************************************************************************/

void AccelerometerXSub::callback( const sensor_msgs::Imu::ConstPtr &msg)
{
	output = msg->linear_acceleration.x;
}

/*******************************************************************************************************/
/*****************                           Accelerometer Y                         *******************/
/*******************************************************************************************************/

void AccelerometerYSub::callback( const sensor_msgs::Imu::ConstPtr &msg)
{
	output = msg->linear_acceleration.y;
}

/*******************************************************************************************************/
/*****************                           Accelerometer Z                         *******************/
/*******************************************************************************************************/

void AccelerometerZSub::callback( const sensor_msgs::Imu::ConstPtr &msg)
{
	output = msg->linear_acceleration.z;
}

/*******************************************************************************************************/
/****************                             NavSatFix                              *******************/
/*******************************************************************************************************/

void NavSatFixSub::setparameters()
{
        if( output.size() != 3 ) throw std::invalid_argument("NavSatFixSub : Output dimension should be 3 !");
        FMatrixSub<sensor_msgs::NavSatFix>::setparameters();
}

void NavSatFixSub::callback( const sensor_msgs::NavSatFix::ConstPtr &msg)
{
	auto mout = getMapVect(output);
	mout[0] = msg->latitude;
	mout[1] = msg->longitude;
	mout[2] = msg->altitude;
}

/*******************************************************************************************************/
/****************                           NavSatFixLat                             *******************/
/*******************************************************************************************************/

void NavSatFixLatSub::callback( const sensor_msgs::NavSatFix::ConstPtr &msg)
{
	output = msg->latitude;
}

/*******************************************************************************************************/
/****************                           NavSatFixLong                            *******************/
/*******************************************************************************************************/

void NavSatFixLongSub::callback( const sensor_msgs::NavSatFix::ConstPtr &msg)
{
	output = msg->longitude;
}

/*******************************************************************************************************/
/****************                           NavSatFixAlt                             *******************/
/*******************************************************************************************************/

void NavSatFixAltSub::callback( const sensor_msgs::NavSatFix::ConstPtr &msg)
{
	output = msg->altitude;
}
