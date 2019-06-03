cmake_minimum_required(VERSION 3.00)
project(StateEstimator)

# Add custom cmake finds
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
# Add macro for subdirectories
include(MacroSubDirList)

set(BUILD_FLAGS "-std=c++11 -Wuninitialized -Wall -Wextra")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${BUILD_FLAGS}")

# fetch all the local directories for generated code
get_filename_component(PROJECT_LIBS_DIR ${PROJECT_BINARY_DIR}/lib ABSOLUTE)
# get_filename_component(SCRIMMAGE_INC_DIR ./include ABSOLUTE)

# Use ccache if available for faster compilations
# see : https://crascit.com/2016/04/09/using-ccache-with-cmake/
find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
  set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CCACHE_PROGRAM}")
else()
  message("ccache not found, repeated builds will be faster if this program is installed")
endif()

# Find and add Boost
find_package(Boost REQUIRED)
find_package(Boost COMPONENTS system filesystem REQUIRED)
include_directories(${Boost_INCLUDE_DIR})
list(APPEND NEEDED_LIBRARIES ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY})

# Find Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
# add_subdirectory(cmake)

# Find tbb
find_package(TBB COMPONENTS tbb tbbmalloc)
if(TBB_FOUND)
	list(APPEND NEEDED_LIBRARIES tbb tbbmalloc)
endif()

# Find GTSAM
find_package(GTSAM REQUIRED)
include_directories(${GTSAM_INCLUDE_DIR})
set(GTSAM_LIBRARIES "/usr/local/lib/libgtsam.so")

# Add our source code
include_directories(./include)
add_subdirectory(src)

# Install all library header files
install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
  )

set(${PROJECT_NAME}_INCLUDE_DIR include)

add_subdirectory(main)

# TODO: build and install all libraries/executables
