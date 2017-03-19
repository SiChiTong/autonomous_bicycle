cmake_minimum_required(VERSION 2.8.3)
project(autonomous_bicycle)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++1y")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        geometry_msgs
        sensor_msgs
        tf
        nav_msgs
        visualization_msgs
        gazebo_ros
        )

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

catkin_package(
        #INCLUDE_DIRS include
        #  LIBRARIES autonomous_bicycle
        CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs sensor_msgs
        #  DEPENDS system_lib
)

include_directories(
        #include
        ${catkin_INCLUDE_DIRS}
)

set(EXECUTABLE_OUTPUT_PATH  ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib )

