cmake_minimum_required(VERSION 3.00)
project(StateEstimator)

set(BUILD_FLAGS "-std=c++11 -Wuninitialized -Wall -Wextra")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${BUILD_FLAGS}")

# Find and add Boost
find_package(Boost REQUIRED)
find_package(Boost COMPONENTS system filesystem REQUIRED)
list(APPEND NEEDED_LIBRARIES ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY})
# Find Eigen
find_package(Eigen3 REQUIRED)
# add_subdirectory(cmake)
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
#find_package(cmake_modules REQUIRED)

find_package(TBB COMPONENTS tbb tbbmalloc)
if(TBB_FOUND)
	list(APPEND NEEDED_LIBRARIES tbb tbbmalloc)
endif()

find_package(GTSAM)
if(GTSAM_FOUND)
    include_directories("/usr/include/eigen3" "/usr/local/include")
    add_executable(${CMAKE_PROJECT_NAME} LocalizationTest.cpp)
    target_link_libraries(${CMAKE_PROJECT_NAME} /usr/local/lib/libgtsam.so)
    target_link_libraries(${CMAKE_PROJECT_NAME} ${NEEDED_LIBRARIES})
endif()

