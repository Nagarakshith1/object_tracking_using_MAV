cmake_minimum_required(VERSION 3.0.2)
project(drone_planner)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
SET(NLOPT_LIBRARY "/usr/local/lib/libnlopt.so")
SET(NLOPT_LIBRARIES "/usr/local/lib/libnlopt.so.0" "/usr/local/lib/libnlopt.so.0.10.0")
SET(NLOPT_INCLUDE_DIRS "/usr/local/include")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  visualization_msgs
  obj_traj_est
)
find_package(Eigen3 REQUIRED)

catkin_package(INCLUDE_DIRS 
  include
  #LIBRARIES drone_planner
  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs nav_msgs visualization_msgs obj_traj_est
  #DEPENDS Eigen3
)

include_directories(include ${EIGEN3_INCLUDE_DIRS} ${NLOPT_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )





## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs#   nav_msgs#   std_msgs#   visualization_msgs
# )

## Declare a C++ library
add_library(genQ src/genQ.cpp)


# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(planner src/planner.cpp)
add_dependencies(planner ${catkin_EXPORTED_TARGETS})
target_link_libraries(planner traj genQ ${NLOPT_LIBRARIES} ${catkin_LIBRARIES})
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")



## Specify libraries to link a library or executable target against



