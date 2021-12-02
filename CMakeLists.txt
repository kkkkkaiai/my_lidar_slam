cmake_minimum_required(VERSION 3.18)
project(lidar_slam)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    sensor_msgs
    nav_msgs
    roscpp
    rospy
    rosbag
    std_msgs
    image_transport
    cv_bridge
    tf
)

find_package(Ceres REQUIRED)
find_package(PCL REQUIRED)
find_package(Boost COMPONENTS thread filesystem date_time system REQUIRED)
find_package(GTSAM REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

INCLUDE_DIRECTORIES(
        include
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIR}
        ${GTSAM_INCLUDE_DIR}
        ${PCL_INCLUDE_DIRS}
        "/usr/include/eigen3"
        ${CERES_INCLUDE_DIR}
        ${YAML_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

catkin_package(
    CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs nav_msgs
    DEPENDS EIGEN3 PCL 
    INCLUDE_DIRS include
)

add_executable(gtsam_demo test/gtsam_demo.cpp)
target_link_libraries(gtsam_demo ${Boost_LIBRARIES} -lgtsam -ltbb)

add_executable(isam_demo test/isam_demo.cpp)
target_link_libraries(isam_demo ${Boost_LIBRARIES} -lgtsam -ltbb)

add_executable(isam_test test/isam_test.cpp)
target_link_libraries(isam_test ${Boost_LIBRARIES} -lgtsam -ltbb)

add_executable(regist src/regist.cpp)
target_link_libraries(regist ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${YAML_CPP_LIBRARIES} gtsam)

add_executable(odometry src/odometry.cpp)
target_link_libraries(odometry ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${YAML_CPP_LIBRARIES} ${CERES_LIBRARIES} gtsam)

add_executable(test_load_param test/test_load_param.cpp)
target_link_libraries(test_load_param ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${YAML_CPP_LIBRARIES} gtsam)

add_executable(mapping src/mapping.cpp)
target_link_libraries(mapping ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${YAML_CPP_LIBRARIES} ${CERES_LIBRARIES} gtsam)

add_executable(isam_pose test/isam_pose3.cpp)
target_link_libraries(isam_pose -lgtsam -ltbb)


