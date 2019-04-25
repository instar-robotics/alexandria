# Copyright (C) INSTAR Robotics
# 
# Author: Pierre Delarboulas
#
# This file is part of alexandria <https://github.com/instar-robotics/alexandria>.
#
# alexandria is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# alexandria is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with dogtag. If not, see <http://www.gnu.org/licenses/>.


cmake_minimum_required(VERSION 3.5.1)

find_package(catkin REQUIRED COMPONENTS kheops)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

project (demofct)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-std=c++17")

add_definitions(-Wextra -Wall)

include_directories(
 ${catkin_INCLUDE_DIRS}
)

add_library(demofct SHARED demofct.cpp)
target_link_libraries(demofct Eigen3::Eigen)


#install(TARGETS demofct DESTINATION "lib"  Eigen3::Eigen )
#PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
