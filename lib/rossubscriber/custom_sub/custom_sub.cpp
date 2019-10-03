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

#include <math.h>
#include "custom_sub.h"

REGISTER_FUNCTION(JointPosSub);
REGISTER_FUNCTION(JointVelSub);
REGISTER_FUNCTION(ObjDetectSub);
REGISTER_FUNCTION(ObjDetectPolarSub);
REGISTER_FUNCTION(AttractorSub);

/*******************************************************************************************************/
/*********************                          JointPos                            ********************/
/*******************************************************************************************************/

void JointPosSub::setparameters()
{
	FMatrixSub<hieroglyph::JointPos>::setparameters();

	if( output.size() != 3 )  throw std::invalid_argument("JointPosSub : Output must be a 3D Vector.");
}

void JointPosSub::callback(const hieroglyph::JointPos::ConstPtr &msg)
{
	auto mout = getMapVect(output);
        mout[0] = msg->accel;
        mout[1] = msg->vel;
        mout[2] = msg->pos;
}


/*******************************************************************************************************/
/*********************                           JointVel                           ********************/
/*******************************************************************************************************/

void JointVelSub::setparameters()
{
	FMatrixSub<hieroglyph::JointVel>::setparameters();

	if( output.size() != 2 )  throw std::invalid_argument("JointVelSub : Output must be a 2D Vector.");
}

void JointVelSub::callback(const hieroglyph::JointVel::ConstPtr &msg)
{
	auto mout = getMapVect(output);
        mout[0] = msg->accel;
        mout[1] = msg->vel;
}

/*******************************************************************************************************/
/*****************                          Object Detection                         *******************/
/*******************************************************************************************************/

void ObjDetectSub::setparameters()
{
        FMatrixSub<hieroglyph::ObjDetect>::setparameters();
        Kernel::iBind(size_x,"size_x", getUuid());
        Kernel::iBind(size_y,"size_y", getUuid());
}

void ObjDetectSub::callback( const hieroglyph::ObjDetect::ConstPtr &msg )
{
        int cloud_pos_x, cloud_pos_y;
        float cloud_val_x, cloud_val_y;
        int eigen_pos_x, eigen_pos_y;

        output = MatrixXd::Constant(output.rows(),output.cols(),0);

        for (unsigned int i=0; i<msg->position.width; i++)
        {
			/** Get point position from PointCloud2 message **/
                cloud_pos_x = i * msg->position.point_step + msg->position.fields[0].offset;
                cloud_pos_y = i * msg->position.point_step + msg->position.fields[1].offset;
                cloud_val_x = *(float*)(&(msg->position.data[cloud_pos_x]));
                cloud_val_y = *(float*)(&(msg->position.data[cloud_pos_y]));

			/** Compute position in matrix from position in cloud **/
                eigen_pos_y = round(cloud_val_x / size_y(0)() * output.rows());
                eigen_pos_x = round((size_x(0)()/2 + cloud_val_y) / size_x(0)() * output.cols()); // Change "+" sign when lidar reverted

			/** Deal with out-of-frame points **/
		if ((eigen_pos_x < output.cols()) && (eigen_pos_y < output.rows()) && (eigen_pos_x >= 0) && (eigen_pos_y >= 0))
			/** Set activity at given position **/
                	output(eigen_pos_y, eigen_pos_x) = msg->confidence[i];
        }
}

/*******************************************************************************************************/
/*****************                       Object Detection Polar                      *******************/
/*******************************************************************************************************/

void ObjDetectPolarSub::setparameters()
{
        FMatrixSub<hieroglyph::ObjDetect>::setparameters();
        Kernel::iBind(size_rho,"size_rho", getUuid());
        Kernel::iBind(size_theta,"size_theta", getUuid());
}

void ObjDetectPolarSub::callback( const hieroglyph::ObjDetect::ConstPtr &msg )
{
        int cloud_pos_x, cloud_pos_y;
        float cloud_val_x, cloud_val_y;
        float cloud_val_rho, cloud_val_theta;
        int eigen_pos_x, eigen_pos_y;

        output = MatrixXd::Constant(output.rows(),output.cols(),0);

        for (unsigned int i=0; i<msg->position.width; i++)
        {
			/** Get point position from PointCloud2 message **/
                cloud_pos_x = i * msg->position.point_step + msg->position.fields[0].offset;
                cloud_pos_y = i * msg->position.point_step + msg->position.fields[1].offset;
                cloud_val_x = *(float*)(&(msg->position.data[cloud_pos_x]));
                cloud_val_y = *(float*)(&(msg->position.data[cloud_pos_y]));

			/** Compute position in matrix from position in cloud **/
                cloud_val_rho = std::sqrt(cloud_val_x*cloud_val_x + cloud_val_y*cloud_val_y);
		cloud_val_theta = atan2(cloud_val_y, cloud_val_x) * 180 / M_PI; // Inverted x and y
                eigen_pos_y = round(cloud_val_rho / size_rho(0)() * output.rows());
                eigen_pos_x = round((cloud_val_theta + size_theta(0)()/2) / size_theta(0)() * output.cols());

			/** Deal with out-of-frame points **/
		if ((eigen_pos_x < output.cols()) && (eigen_pos_y < output.rows()) && (eigen_pos_x >= 0) && (eigen_pos_y >= 0))
			/** Set activity at given position **/
                	output(eigen_pos_y, eigen_pos_x) = msg->confidence[i];
        }
}

/*******************************************************************************************************/
/********************                        Attractor Sub                         *********************/
/*******************************************************************************************************/


void AttractorSub::setparameters()
{
	FMatrixSub<hieroglyph::Attractor>::setparameters();

	if( output.size() != 4 )  throw std::invalid_argument("AttractorSub : Output must be a 4D Vector.");
}

void AttractorSub::callback( const hieroglyph::Attractor::ConstPtr &msg )
{
	auto mout = getMapVect(output);

	mout[0] = msg->norm;
	mout[1] = msg->theta;
	mout[2] = msg->pose_theta;
	mout[3] = msg->force;
}
