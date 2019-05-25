cmake_minimum_required(VERSION 3.00)
project(alpha_pilot)

set(BUILD_FLAGS "-std=c++11 -Wuninitialized -Wall -Wextra")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${BUILD_FLAGS}")

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
#find_package(cmake_modules REQUIRED)


find_package(GTSAM)
if(GTSAM_FOUND)
    #IDK why this is broken
    #find_package(TBB)

    include_directories("/usr/include/eigen3" "/usr/local/include")

    add_executable(StateEstimator LocalizationTest.cpp)
    target_link_libraries(StateEstimator /usr/local/lib/libgtsam.so /usr/lib/x86_64-linux-gnu/libtbb.so /usr/lib/x86_64-linux-gnu/libtbbmalloc.so)

endif()

