cmake_minimum_required(VERSION 2.8.3)
project(bhand_server)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  genmsg
  message_generation
  bhand_gui
)

catkin_package()

include_directories(
  usr/include
  /home/jesus/BHand/API
  /home/jesus/BHand/thirdparty/include
  /home/jesus/BHand/thirdparty/peak-linux-driver-7.14/driver
  /home/jesus/BHand/thirdparty/sources/ctb-0.14/include
  /home/jesus/BHand/thirdparty/sources/poco-1.3.6p2/Foundation/include
  ${catkin_INCLUDE_DIRS}
)

add_executable(bhand_server bhand_server.cpp)
target_link_libraries(bhand_server
  /home/jesus/BHand/API/lib/libBHand.a 
  /home/jesus/BHand/thirdparty/peak-linux-driver-7.14/lib/libpcan.so 
  /usr/local/lib/libctb-barrett-0.14.so
  /usr/local/lib/libPocoFoundationBarrett.so
  ${catkin_LIBRARIES}
)
add_dependencies(
  bhand_server bhand_server_gencpp 
  bhand_message_position bhand_server_generate_messages_cpp 
  bhand_message_velocity bhand_server_generate_messages_cpp 
  bhand_message_feedback bhand_server_generate_messages_cpp
  bhand_message_start_stop bhand_server_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS}
)
