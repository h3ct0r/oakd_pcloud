cmake_minimum_required(VERSION 3.0.2)
project(oakd_pcloud)

#set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
	camera_info_manager
	depthai_ros_msgs
	roscpp
	rospy
	sensor_msgs
	std_msgs
	cv_bridge
	vision_msgs
	depthai_bridge
	message_filters
	genmsg 
	actionlib_msgs 
	actionlib
	dynamic_reconfigure
	visualization_msgs
	image_transport
	dynamic_reconfigure
)

find_package(Boost REQUIRED)
find_package(Eigen3 QUIET)
if(NOT EIGEN3_FOUND)
	find_package(Eigen REQUIRED)
	set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
endif()

set(_opencv_version 4)
find_package(OpenCV 4 QUIET COMPONENTS imgproc highgui)
if(NOT OpenCV_FOUND)
	set(_opencv_version 3)
	find_package(OpenCV 3 REQUIRED COMPONENTS imgproc highgui)
endif()
find_package(depthai REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/WLSfilter.cfg
)

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
	INCLUDE_DIRS 
		include
	LIBRARIES 
		${PROJECT_NAME}
	CATKIN_DEPENDS 
		depthai_ros_msgs 
		roscpp 
		sensor_msgs 
		std_msgs 
		camera_info_manager 
		depthai_bridge 
		vision_msgs 
		cv_bridge message_filters
	DEPENDS
		OpenCV
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${BOOST_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

## Libraries
add_library(${PROJECT_NAME}_lib
	src/stereo_pipeline_lib.cpp
)

add_dependencies(${PROJECT_NAME}_lib
	${${PROJECT_NAME}_EXPORTED_TARGETS} 
	${catkin_EXPORTED_TARGETS}
	${PROJECT_NAME}_gencfg
)

target_link_libraries(${PROJECT_NAME}_lib
  ${catkin_LIBRARIES}
  depthai::core
  opencv_imgproc
  opencv_highgui
  ${OpenCV_LIBRARIES}
  ${BOOST_LIBRARIES}
)

## Nodelets
add_library(${PROJECT_NAME}
	src/nodelet/wls_filter_nodelet.cpp
)

add_dependencies(${PROJECT_NAME}
	${${PROJECT_NAME}_EXPORTED_TARGETS} 
	${catkin_EXPORTED_TARGETS}
	${PROJECT_NAME}_lib
	${PROJECT_NAME}_gencfg
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  depthai::core
  opencv_imgproc
  opencv_highgui
  ${OpenCV_LIBRARIES}
  ${BOOST_LIBRARIES}
  ${PROJECT_NAME}_lib
)

## Nodes
add_executable(stereo_rectified_rgb_node src/stereo_rectified_rgb_publisher.cpp)
    
add_dependencies(stereo_rectified_rgb_node
  ${catkin_EXPORTED_TARGETS} 
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${PROJECT_NAME}_lib
  ${PROJECT_NAME}_gencfg
)

target_link_libraries(stereo_rectified_rgb_node
  ${catkin_LIBRARIES}
  depthai::core
  opencv_imgproc
  opencv_highgui
  ${OpenCV_LIBRARIES}
  ${BOOST_LIBRARIES}
  ${PROJECT_NAME}_lib
)
