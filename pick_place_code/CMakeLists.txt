cmake_minimum_required(VERSION 3.0.2)
project(batu_training)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  actionlib
  actionlib_msgs
  aruco_ros
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  image_transport
  pal_detection_msgs
  cv_bridge
  geometry_msgs
  moveit_msgs
  rospy
  message_generation
  dynamic_reconfigure
  tf
  sensor_msgs
  std_msgs
  std_srvs
  message_generation
)


find_package(OpenCV COMPONENTS opencv_highgui opencv_features2d opencv_xfeatures2d opencv_calib3d)
if(NOT OPENCV_FOUND)
	find_package(OpenCV REQUIRED COMPONENTS opencv_highgui opencv_features2d opencv_calib3d)
	add_definitions(-DNO_CV_XFEATURES2D)
endif()
#list(APPEND OpenCV_LIBRARIES opencv_nonfree)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

generate_dynamic_reconfigure_options(
  cfg/SphericalGrasp.cfg
)

add_message_files(
  FILES
  valueMatrix.msg
  objectLocation.msg
 )

generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  actionlib_msgs geometry_msgs
 )

catkin_package(
  LIBRARIES redObjectDetect 
  CATKIN_DEPENDS roscpp rospy image_transport cv_bridge pal_detection_msgs  geometry_msgs message_generation dynamic_reconfigure sensor_msgs
  DEPENDS system_lib
)
catkin_package(
  LIBRARIES blueObjectDetect 
  CATKIN_DEPENDS roscpp rospy image_transport cv_bridge pal_detection_msgs  geometry_msgs message_generation dynamic_reconfigure sensor_msgs
  DEPENDS system_lib
)

catkin_install_python(PROGRAMS scripts/moverobot.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


include_directories(${catkin_INCLUDE_DIRS})

include_directories(SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(redObjectDetect src/redDetect.cpp)
target_link_libraries(redObjectDetect ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(redObjectDetect ${PROJECT_NAME}_generate_messages_cpp)

add_executable(blueObjectDetect src/blueDetect.cpp)
target_link_libraries(blueObjectDetect ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(blueObjectDetect ${PROJECT_NAME}_generate_messages_cpp)

install(PROGRAMS
        scripts/pick_and_place_server.py
        scripts/pick_client.py
        scripts/spherical_grasps_server.py
        scripts/robotcontrol.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

