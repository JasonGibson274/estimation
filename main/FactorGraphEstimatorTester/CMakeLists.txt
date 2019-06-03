set(3D_PROJECT ${CMAKE_PROJECT_NAME}_tester)

add_executable(${3D_PROJECT} FactorGraphEstimatorTester.cpp)

#TODO should depend on local
target_link_libraries( ${3D_PROJECT}
	${GTSAM_LIBRARIES}
	${NEEDED_LIBRARIES}
	/usr/local/lib/StateEstimator/libFactorGraphEstimator_plugin.so
)
#TODO this should make the other compile
install(TARGETS ${3D_PROJECT} DESTINATION bin)
