add_executable(logger_imu
        main.cpp
)

add_dependencies(logger_imu ${catkin_EXPORTED_TARGETS})

target_include_directories(logger_imu PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} SYSTEM ${catkin_INCLUDE_DIRS})

target_link_libraries(logger_imu
        ${catkin_LIBRARIES}

        atomic
)

install(
        TARGETS logger_imu
        EXPORT logger_imu
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
