cmake_minimum_required(VERSION 2.8.3)
project(rotors_mavlink_interface)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  mavros
  std_msgs
  mavros_msgs
  sensor_msgs
  cmake_modules
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS roscpp mavros mavros_msgs sensor_msgs
  DEPENDS
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(rotors_mavlink_interface src/mavlink_interface.cpp)
add_dependencies(rotors_mavlink_interface ${catkin_EXPORTED_TARGETS})
target_link_libraries(rotors_mavlink_interface ${catkin_LIBRARIES})

install(TARGETS rotors_mavlink_interface
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
