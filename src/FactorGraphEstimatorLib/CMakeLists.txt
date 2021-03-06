#--------------------------------------------------------
# Library Creation
#--------------------------------------------------------
SET (LIBRARY_NAME FactorGraphEstimatorLib)
SET (LIB_MAJOR 0)
SET (LIB_MINOR 0)
SET (LIB_RELEASE 1)

# Don't build if no GTSAM
if(NOT GTSAM_FOUND)
  return()
endif()

file(GLOB SRCS *.cpp)

#FOR Debugging:
#TODO: comment out before committing.
add_definitions (
    -Wfatal-errors
)

ADD_LIBRARY(${LIBRARY_NAME} SHARED
  ${SRCS}
  )

TARGET_LINK_LIBRARIES(${LIBRARY_NAME}
  ${GTSAM_LIBRARIES} ${YAML_CPP_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${GeographicLib_LIBRARY_DIRS}/libGeographic.so
  )

SET (_soversion ${LIB_MAJOR}.${LIB_MINOR}.${LIB_RELEASE})

set_target_properties(${LIBRARY_NAME} PROPERTIES
  SOVERSION ${LIB_MAJOR}
  VERSION ${_soversion}
  LIBRARY_OUTPUT_DIRECTORY ${PROJECT_LIBS_DIR}
)

install(TARGETS ${LIBRARY_NAME}
  # IMPORTANT: Add the library to the "export-set"
  EXPORT ${PROJECT_NAME}-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)

# Push up the PROJECT_PLUGINS variable
set(PROJECT_PLUGINS ${PROJECT_PLUGINS} ${LIBRARY_NAME} PARENT_SCOPE)

