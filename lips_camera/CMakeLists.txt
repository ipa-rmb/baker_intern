cmake_minimum_required(VERSION 2.8.3)
project(lips_camera)

set(catkin_RUN_PACKAGES			# all ROS packages from package.xml (libopencv-dev is system dependency --> sudo apt-get install)
	cv_bridge
	image_transport
	roscpp
	roslib
	sensor_msgs
	std_srvs
	tf
)

set(catkin_BUILD_PACKAGES 
	${catkin_RUN_PACKAGES}
)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	${catkin_BUILD_PACKAGES}		# this makes ${catkin_LIBRARIES} include all libraries of ${catkin_BUILD_PACKAGES}
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)	# name identical to FindOpenCV.cmake in cmake_modules
find_package(PCL REQUIRED)


################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
	INCLUDE_DIRS
		ros/include
#	LIBRARIES lips_camera
	CATKIN_DEPENDS
		${catkin_RUN_PACKAGES}
	DEPENDS
		OpenCV
		PCL
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
	common/include
	ros/include
	${catkin_INCLUDE_DIRS} 		# same name as find_package
	${OpenCV_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(lips_camera
	ros/src/lips_camera_node.cpp
	#ros/src/lips_camera.cpp
)
target_link_libraries(lips_camera
	${catkin_LIBRARIES} # automatically links all catkin_BUILD_PACKAGES
	${OpenCV_LIBRARIES}
	${PCL_LIBRARIES}
	NxLib64.so
)
add_dependencies(lips_camera	${catkin_EXPORTED_TARGETS})


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

## Mark executables and/or libraries for installation
install(TARGETS lips_camera
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
