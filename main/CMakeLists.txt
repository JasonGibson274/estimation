set(2D_PROJECT ${CMAKE_PROJECT_NAME}_2d)
set(3D_PROJECT ${CMAKE_PROJECT_NAME}_3d)
add_executable(${2D_PROJECT} LocalizationTest.cpp)
target_link_libraries(${2D_PROJECT} ${GTSAM_LIBRARIES}))
target_link_libraries(${2D_PROJECT} ${NEEDED_LIBRARIES})

install(TARGETS ${2D_PROJECT} DESTINATION bin)

add_executable(${3D_PROJECT} Imu3dTest.cpp)
target_link_libraries(${3D_PROJECT} ${GTSAM_LIBRARIES})
target_link_libraries(${3D_PROJECT} ${NEEDED_LIBRARIES})
install(TARGETS ${3D_PROJECT} DESTINATION bin)