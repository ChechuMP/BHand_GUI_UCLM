cmake_minimum_required(VERSION 2.8.3)
project(bhand_joint)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  genmsg
  bhand_gui
)

catkin_package()

include_directories(
  usr/include
  ${catkin_INCLUDE_DIRS}
)

add_executable(bhand_joint bhand_joint.cpp)

target_link_libraries(
  bhand_joint 
  ${catkin_LIBRARIES}
)

add_dependencies(
  bhand_joint bhand_joint_gencpp
  bhand_message_simulation bhand_joint_generate_messages_cpp 
  ${catkin_EXPORTED_TARGETS}
)
