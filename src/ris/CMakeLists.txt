cmake_minimum_required(VERSION 3.10)
project(ris)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_VERBOSE_MAKEFILE ON)

add_compile_options(-fPIC -Wall -pedantic)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(
    catkin REQUIRED COMPONENTS
    roscpp
    roslib
    rospy
    ris_msgs
    std_msgs
    message_runtime
)

catkin_package(
    CATKIN_DEPENDS message_runtime ris_msgs
)

catkin_install_python(PROGRAMS
        scripts/lidar_listener.py
        scripts/publisher_node.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_subdirectory(src)
#add_subdirectory(scripts_py)
#add_subdirectory(data)
#add_subdirectory(launch)
