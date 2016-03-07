cmake_minimum_required(VERSION 2.8.3)
project(bhand_gui)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rqt_gui
  rqt_gui_cpp
  pluginlib
  genmsg
  message_generation
)

catkin_python_setup()

add_message_files(
  FILES
  bhand_message_simulation.msg
  bhand_message_position.msg
  bhand_message_velocity.msg
  bhand_message_feedback.msg
  bhand_message_start_stop.msg
)

generate_messages(
  DEPENDENCIES
  bhand_gui
)

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)

include(${QT_USE_FILE})

set(bhand_gui_SRCS
  src/bhand_gui/bhand_gui.cpp
)

set(bhand_gui_HDRS
  include/bhand_gui/bhand_gui.h
)

set(bhand_gui_UIS
  resource/bhand_gui/bhand_gui.ui
)

set(bhand_gui_INCLUDE_DIRECTORIES
  include
  ${CMAKE_CURRENT_BINARY_DIR}
)

catkin_package()

qt4_wrap_cpp(bhand_gui_MOCS ${bhand_gui_HDRS})
qt4_wrap_ui(bhand_gui_UIS_H ${bhand_gui_UIS})

include_directories(
  usr/include 
  ${bhand_gui_INCLUDE_DIRECTORIES} 
  ${catkin_INCLUDE_DIRS}
)

add_library(
  ${PROJECT_NAME} 
  ${bhand_gui_SRCS} 
  ${bhand_gui_MOCS} 
  ${bhand_gui_UIS_H}
)

target_link_libraries(
  ${PROJECT_NAME} 
  ${catkin_LIBRARIES} 
  ${QT_QTCORE_LIBRARY} 
  ${QT_QTGUI_LIBRARY}
)

