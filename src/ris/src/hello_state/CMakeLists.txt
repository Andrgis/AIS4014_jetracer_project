add_executable(hello_state
        main.cpp
)

add_dependencies(hello_state ${catkin_EXPORTED_TARGETS})

target_include_directories(hello_state PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} SYSTEM ${catkin_INCLUDE_DIRS})

target_link_libraries(hello_state
        ${catkin_LIBRARIES}

        atomic
)

install(
        TARGETS hello_state
        EXPORT hello_state
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
