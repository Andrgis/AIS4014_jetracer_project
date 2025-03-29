add_executable(learning_publisher
        main.cpp
)

add_dependencies(learning_publisher ${catkin_EXPORTED_TARGETS})

target_include_directories(learning_publisher PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} SYSTEM ${catkin_INCLUDE_DIRS})

target_link_libraries(learning_publisher
        ${catkin_LIBRARIES}

        atomic
)

install(
        TARGETS learning_publisher
        EXPORT learning_publisher
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
