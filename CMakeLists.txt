cmake_minimum_required(VERSION 3.5.1)
project(alexandria)

set(CATKIN_DEP
  kheops
  joy
  tf
  nav_msgs
  sensor_msgs
  pcl_ros
)

find_package(catkin REQUIRED COMPONENTS ${CATKIN_DEP})
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

###################################
## catkin specific configuration ##
###################################

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
  CATKIN_DEPENDS ${CATKIN_DEP}
#  DEPENDS
#  LIBRARIES
)

###########
## Build ##
###########

set(CMAKE_CXX_FLAGS "-std=c++17 -O2 -msse2 -mtune=native -march=native")
add_definitions(-Wextra -Wall)
file(GLOB_RECURSE sourcefiles lib/*.cpp)


include_directories(
 ${catkin_INCLUDE_DIRS}
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/${PROJECT_NAME}")
set(SHARE_DESC "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/descriptions")

  FOREACH(src ${sourcefiles})
    GET_FILENAME_COMPONENT(basename ${src} NAME_WE) 
    file(RELATIVE_PATH relative_path ${CMAKE_CURRENT_SOURCE_DIR}/lib ${src})
    GET_FILENAME_COMPONENT(basedir ${relative_path} DIRECTORY)
     
    ADD_LIBRARY(${basename} SHARED ${src}  )
    target_link_libraries(${basename} Eigen3::Eigen ${catkin_LIBRARIES})
    

    set(xml "${CMAKE_CURRENT_SOURCE_DIR}/lib/${basedir}/${basename}.xml")
    add_custom_command(TARGET ${basename} PRE_BUILD 
    		    COMMAND mkdir -p ${SHARE_DESC}/${basedir}
    		    COMMAND cp ${xml}  ${SHARE_DESC}/${basedir}
    		    COMMENT "Copying XML File description for library ${basename}"
    		    )
	    
    GET_FILENAME_COMPONENT(icon_base_dir ${src} DIRECTORY)
    if(NOT EXISTS "${icon_base_dir}/icons")
    	    message(STATUS "Create empty Icon directory : ${icon_base_dir}/icons ")
    	    file(MAKE_DIRECTORY "${icon_base_dir}/icons")
    endif()	     
		    
    add_custom_command(TARGET ${basename} PRE_BUILD
                   COMMAND mkdir -p ${SHARE_DESC}/${basedir}
                   COMMAND cp -r "${icon_base_dir}/icons"  ${SHARE_DESC}/${basedir}
                   COMMENT "Copying icon File description for library ${basename}"
		   )
    ENDFOREACH()
	  
    add_custom_target("clean_desc" 
    	     COMMAND rm -rf ${SHARE_DESC}/*
	     )
    add_custom_target("all_desc")
    
    add_dependencies("all_desc" "clean_desc")

    file(GLOB_RECURSE xmlfiles lib/*.xml) 
    FOREACH(xml ${xmlfiles})
    	  GET_FILENAME_COMPONENT(basename ${xml} NAME_WE) 
    	  file(RELATIVE_PATH relative_path ${CMAKE_CURRENT_SOURCE_DIR}/lib ${xml})
    	  GET_FILENAME_COMPONENT(basedir ${relative_path} DIRECTORY)
     	  GET_FILENAME_COMPONENT(icon_base_dir ${xml} DIRECTORY)
    	    
    	  add_custom_command(OUTPUT ${SHARE_DESC}/${basedir}/${xml}
    		    COMMAND mkdir -p ${SHARE_DESC}/${basedir}
    		    COMMAND cp ${xml}  ${SHARE_DESC}/${basedir}
                    COMMAND cp -r "${icon_base_dir}/icons"  ${SHARE_DESC}/${basedir}
    		    DEPENDS ${xml}
		    COMMENT "Copying XML and icons File description for library ${basename}"
    		    )

    	  add_custom_target("${basename}_desc" 
		  DEPENDS ${SHARE_DESC}/${basedir}/${xml}
		  )
    	  
	  add_dependencies("${basename}_desc" "clean_desc")
	  add_dependencies("all_desc"  "${basename}_desc")
    ENDFOREACH()

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# INSTALL_SHARED_LIBRARY( ${basename} DESTINATION lib )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
#install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_kheops.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
