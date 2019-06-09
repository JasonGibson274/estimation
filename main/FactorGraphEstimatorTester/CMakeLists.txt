set(3D_PROJECT ${CMAKE_PROJECT_NAME}_tester)

# Don't build if no GTSAM
if(NOT GTSAM_FOUND)
  return()
endif()

add_executable(${3D_PROJECT} FactorGraphEstimatorTester.cpp)

target_link_libraries( ${3D_PROJECT}
	${GTSAM_LIBRARIES}
	${NEEDED_LIBRARIES}
	FactorGraphEstimator_plugin
)

install(TARGETS ${3D_PROJECT} DESTINATION bin)
