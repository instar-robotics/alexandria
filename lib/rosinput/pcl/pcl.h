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


#ifndef __PCL_HPP__
#define __PCL_HPP__

#include "kheops/kernel/function.h"
#include "kheops/kernel/kernel.h"
#include "kheops/ros/rossubscriber.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>


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

#endif // __PCL_HPP__

