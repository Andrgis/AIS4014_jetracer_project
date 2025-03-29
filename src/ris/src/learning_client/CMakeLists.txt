add_executable(learning_client
        main.cpp
)

add_dependencies(learning_client ${catkin_EXPORTED_TARGETS})

target_include_directories(learning_client PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} SYSTEM ${catkin_INCLUDE_DIRS})

target_link_libraries(learning_client
        ${catkin_LIBRARIES}

        atomic
)

install(
        TARGETS learning_client
        EXPORT learning_client
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
