cmake_minimum_required(VERSION 2.8.3)
project(laser_mapping)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(catkin REQUIRED 
roscpp rospy rosbag std_msgs geometry_msgs nav_msgs
tf tf2 tf_conversions message_filters map_msgs  visualization_msgs
interactive_markers  message_generation roslib
libpointmatcher_ros
)
find_package(libnabo)

find_package(Eigen3 )

find_package(Glog REQUIRED)

find_package(Boost COMPONENTS thread) 

find_package(libpointmatcher REQUIRED)

add_service_files( FILES
CorrectPose.srv
GetBoundedMap.srv
GetMode.srv
LoadMap.srv
MatchClouds.srv
SetMode.srv
)

add_message_files(
FILES
Localizing_Mode.msg
)


generate_messages(DEPENDENCIES sensor_msgs nav_msgs std_msgs geometry_msgs actionlib_msgs)


catkin_package (
	CATKIN_DEPENDS roscpp rospy rosbag std_msgs geometry_msgs nav_msgs
tf tf2 tf_conversions message_filters map_msgs   visualization_msgs interactive_markers 
message_runtime roslib

)

# set(CMAKE_BUILD_TYPE RelWithDebInfo)
set(CMAKE_BUILD_TYPE Release)

#else some functions cannot be used
if(CMAKE_COMPILER_IS_GNUCXX)
	execute_process(COMMAND ${CMAKE_C_COMPILER} -dumpversion OUTPUT_VARIABLE GCC_VERSION)
	if (GCC_VERSION VERSION_GREATER 4.4 OR GCC_VERSION VERSION_EQUAL 4.4)
		add_definitions(-std=gnu++0x)
	else (GCC_VERSION VERSION_GREATER 4.4 OR GCC_VERSION VERSION_EQUAL 4.4)
		message(SEND_ERROR "You need GCC version 4.4 or greater to compile this library")
	endif (GCC_VERSION VERSION_GREATER 4.4 OR GCC_VERSION VERSION_EQUAL 4.4)
else(CMAKE_COMPILER_IS_GNUCXX)
	message(SEND_ERROR "You need partial C++0x support for N2351, N2672, N1984. Currently this build toolchain supports only GCC >= 4.4. If your compiler supports these specifications, please send us a patch.")
endif(CMAKE_COMPILER_IS_GNUCXX)



add_definitions(-DHAVE_YAML_CPP)
add_definitions(-std=c++11 -O2 -g -DNDEBUG)

include_directories(
include
include/vtr
${PROJECT_SOURCE_DIR}
${catkin_INCLUDE_DIRS}
${libpointmatcher_INCLUDE_DIRS}
${EIGEN3_INCLUDE_DIR}
${GLOG_INCLUDE_DIRS}
${PROJECT_SOURCE_DIR}/Thirdparty/sophus
 /usr/local/include/EXTERNAL/csparse
 ${Boost_INCLUDE_DIRS}
#/usr/include/suitesparse

)

add_library(g2o SHARED IMPORTED)
set_property(TARGET g2o PROPERTY IMPORTED_LOCATION "${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so")
file(
  COPY ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
  DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}
)

set(LIB LaserMappingLib)

add_library(${LIB} SHARED

src/IMU/configparam.cpp
src/IMU/imudata.cpp
src/IMU/IMUPreintegrator.cpp
src/IMU/NavState.cpp
src/IMU/g2otypes.cpp
src/imu_preintegration.cpp
src/preintegration_opt.cc

src/odom_preintegration.cpp
src/Odom/odomdata.cpp
src/Odom/OdomPreintegrator.cpp


src/Frame.cpp
src/KeyFrame.cpp
src/icp_mapper.cpp
src/LocalMapping.cpp
src/Map.cpp
src/MapPublisher.cpp
src/Optimizer.cpp
src/LoopClosing.cpp
src/GPS.cpp
)


target_link_libraries(${LIB}
${catkin_LIBRARIES}
${EIGEN3_LIBS}
${GLOG_LIBRARIES}
${Boost_LIBRARIES}
${libpointmatcher_LIBRARIES}
g2o
)


add_executable(laser_mapping_with_bag
src/laser_mapping_with_bag.cpp)

add_dependencies(laser_mapping_with_bag ${PROJECT_NAME}_gencpp)
target_link_libraries(laser_mapping_with_bag
${LIB}
)


add_executable(mapping_by_tf
src/mapping_by_tf.cpp)

add_dependencies(mapping_by_tf ${PROJECT_NAME}_gencpp)
target_link_libraries(mapping_by_tf
${LIB}
)




## Mark executables and/or libraries for installation
#install(TARGETS dynamic_SLAM localization
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

# Mark other files for installation
#install(
#  DIRECTORY cfg launch
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)

#install(
#  FILES ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
#  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#)

