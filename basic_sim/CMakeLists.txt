cmake_minimum_required(VERSION 2.8.3)
project(basic_sim)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  controller_manager
  hardware_interface
  urdf
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ros_control_sim
  EXECUTABLES sim_node
  CATKIN_DEPENDS roscpp controller_manager hardware_interface urdf
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_library(ros_control_sim
  src/generic_hw_interface.cpp
  src/sim_hw_interface.cpp
)
target_link_libraries(ros_control_sim ${catkin_LIBRARIES})
add_executable(sim_node src/sim_node.cpp)
target_link_libraries(sim_node ros_control_sim)
