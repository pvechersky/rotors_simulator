cmake_minimum_required(VERSION 2.8.3)
project(rotors_hil_interface)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  mavros
  mav_msgs
  std_msgs
  mavros_msgs
  sensor_msgs
  cmake_modules
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS roscpp mav_msgs mavros mavros_msgs sensor_msgs std_msgs
  DEPENDS
)

include_directories(include ${catkin_INCLUDE_DIRS} ${mavros_msgs_INCLUDE_DIRS})

add_executable(rotors_hil_control_interface src/hil_control_interface.cpp)
add_dependencies(rotors_hil_control_interface ${catkin_EXPORTED_TARGETS} ${mavros_EXPORTED_TARGETS} ${mavros_msgs_EXPORTED_TARGETS})
target_link_libraries(rotors_hil_control_interface ${catkin_LIBRARIES} ${mavros_LIBRARIES})

add_executable(rotors_hil_sensors_interface src/hil_sensors_interface.cpp)
add_dependencies(rotors_hil_sensors_interface ${catkin_EXPORTED_TARGETS} ${mavros_EXPORTED_TARGETS} ${mavros_msgs_EXPORTED_TARGETS})
target_link_libraries(rotors_hil_sensors_interface ${catkin_LIBRARIES} ${mavros_LIBRARIES})

install(TARGETS
    rotors_hil_control_interface
    rotors_hil_sensors_interface
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
