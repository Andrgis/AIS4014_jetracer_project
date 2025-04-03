add_executable(logger_cam
        main.cpp
)

add_dependencies(logger_cam ${catkin_EXPORTED_TARGETS})

target_include_directories(logger_cam PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} SYSTEM ${catkin_INCLUDE_DIRS})

target_link_libraries(logger_cam
        ${catkin_LIBRARIES}

        atomic
)

install(
        TARGETS logger_cam
        EXPORT logger_cam
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
