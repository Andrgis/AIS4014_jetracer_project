add_executable(learning_server
        main.cpp
)

add_dependencies(learning_server ${catkin_EXPORTED_TARGETS})

target_include_directories(learning_server PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} SYSTEM ${catkin_INCLUDE_DIRS})

target_link_libraries(learning_server
        ${catkin_LIBRARIES}

        atomic
)

install(
        TARGETS learning_server
        EXPORT learning_server
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
