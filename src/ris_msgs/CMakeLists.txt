cmake_minimum_required(VERSION 3.10)

project(ris_msgs)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_VERBOSE_MAKEFILE ON)

set(MESSAGE_DEPENDENCIES
    std_msgs
)

find_package(catkin REQUIRED genmsg ${MESSAGE_DEPENDENCIES})

add_message_files(DIRECTORY msg/ FILES
        Hello.msg
	Dog_data.msg
)

add_service_files(DIRECTORY srv/ FILES
        PingPong.srv
	Addition_service_node.srv
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

generate_messages(DEPENDENCIES ${MESSAGE_DEPENDENCIES})

catkin_package(DEPENDS ${MESSAGE_DEPENDENCIES})



