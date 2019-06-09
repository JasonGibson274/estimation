set(2D_PROJECT ${CMAKE_PROJECT_NAME}_2d)

# Don't build if no GTSAM
if(NOT GTSAM_FOUND)
  return()
endif()

add_executable(${2D_PROJECT} LocalizationTest.cpp)

target_link_libraries( ${2D_PROJECT}
	${GTSAM_LIBRARIES}
 	${NEEDED_LIBRARIES}
)

install(TARGETS ${2D_PROJECT} DESTINATION bin)