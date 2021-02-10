# Don't build if no GTSAM
if(NOT GTSAM_FOUND)
  return()
endif()

add_executable(factor_graph_estimator_node FactorGraphEstimatorNode.cpp)
target_link_libraries(factor_graph_estimator_node ${ROS_LIBRARIES}
        ${GTSAM_LIBRARIES} ${YAML_CPP_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${GeographicLib_LIBRARY_DIRS}/libGeographic.so
        FactorGraphEstimatorLib)

