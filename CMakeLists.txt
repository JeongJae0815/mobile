cmake_minimum_required(VERSION 2.8.3)
project(mobile)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  std_msgs
#above packages is for serial communication
  geometry_msgs 
  rosaria
  rospy
#above packages is for controlling mobile robot
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  DEPENDS system_lib
  CATKIN_DEPENDS serial std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(serial_example_node src/serial_example_node.cpp)
add_executable(mobile_inf src/interface.cpp)
add_executable(mobile_ctrl src/controller.cpp)

target_link_libraries(serial_example_node
  ${catkin_LIBRARIES}
)

target_link_libraries(mobile_inf
  ${catkin_LIBRARIES}
)

target_link_libraries(mobile_ctrl
  ${catkin_LIBRARIES}
)


