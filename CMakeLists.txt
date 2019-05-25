cmake_minimum_required(VERSION 3.00)
project(StateEstimator)
set(2D_PROJECT ${CMAKE_PROJECT_NAME}_2d)
set(3D_PROJECT ${CMAKE_PROJECT_NAME}_3d)
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
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(TBB COMPONENTS tbb tbbmalloc)
if(TBB_FOUND)
	list(APPEND NEEDED_LIBRARIES tbb tbbmalloc)
endif()

find_package(GTSAM)
if(GTSAM_FOUND)
    include_directories(${GTSAM_INCLUDE_DIR})
    add_executable(${2D_PROJECT} LocalizationTest.cpp)
    target_link_libraries(${2D_PROJECT} /usr/local/lib/libgtsam.so)
    target_link_libraries(${2D_PROJECT} ${NEEDED_LIBRARIES})

    add_executable(${3D_PROJECT} Imu3dTest.cpp)
    target_link_libraries(${3D_PROJECT} /usr/local/lib/libgtsam.so)
    target_link_libraries(${3D_PROJECT} ${NEEDED_LIBRARIES})
endif()

