add_executable(logger_odom
        main.cpp
)

add_dependencies(logger_odom ${catkin_EXPORTED_TARGETS})

target_include_directories(logger_odom PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} SYSTEM ${catkin_INCLUDE_DIRS})

target_link_libraries(logger_odom
        ${catkin_LIBRARIES}

        atomic
)

install(
        TARGETS logger_odom
        EXPORT logger_odom
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
