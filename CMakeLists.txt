cmake_minimum_required(VERSION 3.0.2)
project(robot_posture_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  #LIBRARIES om_modbus_master
  
  # add
  CATKIN_DEPENDS roscpp rospy std_msgs

)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(robot_posture_control src/compute_kinematics.cpp src/compute_motor_angle.cpp src/robot_posture_control_node.cpp)
target_link_libraries(robot_posture_control ${catkin_LIBRARIES})