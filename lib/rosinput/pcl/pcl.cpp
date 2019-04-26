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

#include "pcl.h"

REGISTER_FUNCTION(Lidar2D);

/*******************************************************************************************************/
/************************                       Lidar2D                        *************************/
/*******************************************************************************************************/

void Lidar2D::compute()
{
        callOne(sleep()());
}

void Lidar2D::setparameters()
{
        Kernel::iBind(topic_name,"topic_name", getUuid());
        Kernel::iBind(size_queue,"size_queue", getUuid());
        Kernel::iBind(sleep,"sleep", getUuid());
        Kernel::iBind(range_max,"range_max", getUuid());

        moy = MatrixXi::Constant(output.rows(), output.cols(),0);
}

//void Lidar2D::callback(const sensor_msgs::PointCloud2::ConstPtr &msg )
void Lidar2D::callback(const PointCloud::ConstPtr &msg )
{
        output = MATRIX::Constant(output.rows(),output.cols(),0);

        for( unsigned int i = 0 ; i <  msg->points.size() ; i++ )
        {


                 SCALAR theta = (2 * atan( msg->points[i].y / ( msg->points[i].x + sqrt( msg->points[i].x * msg->points[i].x + msg->points[i].y * msg->points[i].y   ))) + M_PI ) *  output.cols() / (2*M_PI);

                 SCALAR value = sqrt( msg->points[i].x * msg->points[i].x + msg->points[i].y * msg->points[i].y + msg->points[i].z * msg->points[i].z  );

                 std::cout <<  msg->points[i].z+10  << " " << theta << " "<< value << std::endl;
                 if( ! std::isnan(theta) )  output( msg->points[i].z+10 , theta) = value;
        }

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

void Lidar2D::onQuit()
{
        disable();
}

void Lidar2D::onPause()
{
        disable();
}

void Lidar2D::onRun()
{
        enable( topic_name, (int)(size_queue()())   );
}

