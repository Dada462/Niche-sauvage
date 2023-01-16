cmake_minimum_required(VERSION 3.0.2)
project(bluerov_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geographic_msgs
  sensor_msgs
  visualization_msgs
  geometry_msgs
  nav_msgs
)

include_directories(include)

add_message_files(
  DIRECTORY msg
  FILES
  CommandBluerov.msg
)


generate_messages(DEPENDENCIES geographic_msgs geometry_msgs sensor_msgs std_msgs)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")
