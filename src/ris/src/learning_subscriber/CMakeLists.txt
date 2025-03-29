add_executable(learning_subscriber
        main.cpp
)

add_dependencies(learning_subscriber ${catkin_EXPORTED_TARGETS})

target_include_directories(learning_subscriber PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} SYSTEM ${catkin_INCLUDE_DIRS})

target_link_libraries(learning_subscriber
        ${catkin_LIBRARIES}

        atomic
)

install(
        TARGETS learning_subscriber
        EXPORT learning_subscriber
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
