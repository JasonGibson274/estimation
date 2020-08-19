# Don't build if no GTSAM
if(NOT GTSAM_FOUND)
  return()
endif()

#add_executable(FactorGraphEstimatorNode FactorGraphEstimatorNode.cpp)
#target_link_libraries(FactorGraphEstimatorLib ${catkin_LIBRARIES} ${ROS_LIBRARIES}
#        gtsam ${GeographicLib_LIBRARY_DIRS}/libGeographic.so ${TBB_LIBRARIES} ${Boost_LIBRARIES})

