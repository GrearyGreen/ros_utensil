cmake_minimum_required(VERSION 3.0.2)
project(ros_utensil)

set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_CXX_FLAGS "-std=c++11")
# 设置 C++ 编译标准
set(CMAKE_CXX_STANDARD 14)  # 使用 C++14 标准
set(CMAKE_CXX_STANDARD_REQUIRED True)
# set(CMAKE_CXX_FLAGS "-std=c++14")
# 编译已经启用多线程支持
# pthread
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")


find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    pcl_conversions
    pcl_ros
    roscpp
    rospy
    sensor_msgs
    std_msgs
)
find_package(PCL REQUIRED)
# Boost 文件系统库
find_package(Boost REQUIRED COMPONENTS filesystem regex)

catkin_package(
    INCLUDE_DIRS include
    DEPENDS PCL 
    # GTSAM

    CATKIN_DEPENDS
    std_msgs
    # nav_msgs
    geometry_msgs
    sensor_msgs
    # message_runtime
    # message_generation
    # visualization_msgs
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

link_directories(
	include
	${PCL_LIBRARY_DIRS}
    # ${OpenCV_LIBRARY_DIRS}
    # ${GTSAM_LIBRARY_DIRS}
)

add_executable(bag_to_pcd src/bag_to_pcd.cpp)
# 添加可执行文件
add_executable(pcd_to_bag src/pcd_to_bag.cpp)

target_link_libraries(bag_to_pcd
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${Boost_LIBRARIES}
)

# 链接库
target_link_libraries(pcd_to_bag
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PCL_LIBRARIES}
#   stdc++fs
    # Boost::Boost 
    Boost::regex
)
