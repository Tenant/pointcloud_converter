cmake_minimum_required(VERSION 3.5)

project(pointcloud_converter LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Qt5Core REQUIRED)
find_package(Qt5Xml REQUIRED)
find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        sensor_msgs
        std_msgs
        message_generation
        rosbag
        pcl_ros
        tf)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${Qt5Core_INCLUDE_DIRS}
    ${Qt5Xml_INCLUDE_DIRS}
    .
    include)

aux_source_directory(src DIR_ROOT_SRC)

add_executable(pointcloud_converter
  main.cpp
  mainwindow.cpp
  mainwindow.h
  mainwindow.ui
  ${DIR_ROOT_SRC})

target_link_libraries(pointcloud_converter
    PRIVATE Qt5::Widgets
    ${catkin_LIBRARIES}
    ${Qt5Core_LIBRARIES}
    ${Qt5Xml_LIBRARIES})
