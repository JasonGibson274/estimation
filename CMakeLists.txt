cmake_minimum_required(VERSION 3.00)
project(StateEstimator)

set(BUILD_FLAGS "-std=c++11 -Wuninitialized -Wall -Wextra")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${BUILD_FLAGS}")

# Find and add Boost
find_package(Boost REQUIRED)
find_package(Boost COMPONENTS system filesystem REQUIRED)
include_directories(${Boost_INCLUDE_DIR})
list(APPEND NEEDED_LIBRARIES ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY})

# Find Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
# add_subdirectory(cmake)

# Add custom cmake finds
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

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
add_subdirectory(main)
add_subdirectory(src)

# Install all library header files
install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
  )

# TODO: build and install all libraries/executables
