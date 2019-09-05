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

#ifndef __NAV_PUB_HPP__
#define __NAV_PUB_HPP__

#include "kheops/ros/fpub.h"
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

/*
 * Path : Send path parameters in nav_msgs/Path (only one pose could be send)
 * position : a 3D Vector
 * orientation : a 3D Vector (Euler angle, will be convert to Quaternion)
 * Output dimension should be 6 (3 Scalar for pos and 3 for orientation)
 */
class PathPub :  public FMatrixPub<nav_msgs::Path>
{
        private :

                IString frame_id;
                ISMInput position;
                ISMInput orientation;

               nav_msgs::Path msg;

        public :

                PathPub() : FMatrixPub<nav_msgs::Path>(VECTOR) {}
                virtual ~PathPub(){}

                virtual void compute();
                virtual void setparameters();
                virtual void prerun();
};

#endif // __NAV_PUB_HPP__
