cmake_minimum_required(VERSION 3.0.2)
project(obj_traj_est)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  pcl_ros
  std_msgs
)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})


catkin_package(INCLUDE_DIRS 
  #include
  #LIBRARIES obj_traj_est
  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs pcl_ros
  #DEPENDS Eigen
)


include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_library(k_filter src/k_filter.cpp)

add_executable(test_eigen src/test_eigen.cpp)
#add_dependencies(test_eigen ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_eigen k_filter ${catkin_LIBRARIES})