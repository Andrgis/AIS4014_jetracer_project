add_executable(position_estimator
        main.cpp
)

add_dependencies(position_estimator ${catkin_EXPORTED_TARGETS})

target_include_directories(position_estimator PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} SYSTEM ${catkin_INCLUDE_DIRS})

target_link_libraries(position_estimator
        ${catkin_LIBRARIES}

        atomic
)

install(
        TARGETS position_estimator
        EXPORT position_estimator
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
