cmake_minimum_required(VERSION 2.8.3)
project(reflex_gripper_rr_bridge)

find_package(catkin REQUIRED COMPONENTS
  reflex_gripper
  rospy
  std_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS 
  RobotRaconteur 
  reflex_gripper 
  rospy 
  std_msgs
)

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)
