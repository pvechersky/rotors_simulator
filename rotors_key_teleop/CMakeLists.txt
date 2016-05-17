cmake_minimum_required(VERSION 2.8.3)
project(rotors_key_teleop)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)
include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
  DEPENDS
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp
  LIBRARIES
)

add_definitions(-std=c++11)

add_executable(rotors_key_teleop src/key_teleop.cpp)
add_dependencies(rotors_key_teleop ${catkin_EXPORTED_TARGETS})
target_link_libraries(rotors_key_teleop ${catkin_LIBRARIES})

install(TARGETS rotors_key_teleop
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
