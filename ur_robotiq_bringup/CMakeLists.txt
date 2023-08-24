cmake_minimum_required(VERSION 2.8.3)
project(ur_robotiq_bringup)

add_compile_options(-std=c++14)

## Declare a catkin package
catkin_package()

roslaunch_add_file_check(launch)

## Install ## 
install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


