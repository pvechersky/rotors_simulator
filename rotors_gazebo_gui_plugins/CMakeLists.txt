cmake_minimum_required(VERSION 2.8.3)
project(rotors_gazebo_gui_plugins)

add_definitions(-std=c++11)

# To enable assertions when compiled in release mode.
add_definitions(-DROS_ASSERT_ENABLED)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
)

find_package(gazebo REQUIRED)
find_package(Protobuf REQUIRED)
find_package(Qt4 REQUIRED)

include (${QT_USE_FILE})
add_definitions(${QT_DEFINITIONS})

link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(include
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
  CATKIN_DEPENDS roscpp
  DEPENDS gazebo
)

QT4_WRAP_CPP(headers_view_control include/rotors_gazebo_gui_plugins/gazebo_gui_view_control_plugin.h)
QT4_WRAP_CPP(headers_point_coordinates include/rotors_gazebo_gui_plugins/gazebo_gui_rotors_toolbar_plugin.h)

add_library(rotors_gazebo_gui_view_control_plugin SHARED ${headers_view_control} src/gazebo_gui_view_control_plugin.cpp)
target_link_libraries(rotors_gazebo_gui_view_control_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${QT_LIBRARIES} ${PROTOBUF_LIBRARIES})
add_dependencies(rotors_gazebo_gui_view_control_plugin ${catkin_EXPORTED_TARGETS})

add_library(rotors_gazebo_gui_rotors_toolbar_plugin SHARED ${headers_point_coordinates} src/gazebo_gui_rotors_toolbar_plugin.cpp)
target_link_libraries(rotors_gazebo_gui_rotors_toolbar_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${QT_LIBRARIES} ${PROTOBUF_LIBRARIES})
add_dependencies(rotors_gazebo_gui_rotors_toolbar_plugin ${catkin_EXPORTED_TARGETS})

install(
  TARGETS
    rotors_gazebo_gui_view_control_plugin
    rotors_gazebo_gui_rotors_toolbar_plugin
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

