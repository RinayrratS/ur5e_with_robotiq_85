cmake_minimum_required(VERSION 3.0.2)
project(robotiq_gripper_controller)


find_package(catkin REQUIRED COMPONENTS
  robotiq_gripper_msgs
  roscpp
  sensor_msgs
)


## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
)



## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)



add_executable(${PROJECT_NAME}_node src/main_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  modbus
)

