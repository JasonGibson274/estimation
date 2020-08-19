cmake_minimum_required(VERSION 3.10)
project(autorally_estimation)

# Add custom cmake finds
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")

# Add macro for subdirectories
include(MacroSubDirList)
# Fix needed for linaro gcc compiler according to https://gcc.gnu.org/onlinedocs/gcc/Directory-Options.html
set(CMAKE_INCLUDE_SYSTEM_FLAG_CXX "-I")
message("WE ARE USING ${CMAKE_INCLUDE_SYSTEM_FLAG_CXX}")

set(BUILD_FLAGS "-std=c++11 -Wuninitialized -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${BUILD_FLAGS}")

# Allow the lib location to be overwritten from command line
if (NOT CMAKE_LIBRARY_OUTPUT_DIRECTORY)
	get_filename_component(PROJECT_LIBS_DIR ${PROJECT_BINARY_DIR}/lib ABSOLUTE)
	set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
	message("${PROJECT_LIBS_DIR}")
else()
	get_filename_component(PROJECT_LIBS_DIR ${CMAKE_LIBRARY_OUTPUT_DIRECTORY} ABSOLUTE)
endif()

# Find and add Boost
find_package(Boost REQUIRED)
find_package(Boost COMPONENTS system filesystem thread serialization date_time timer regex REQUIRED)
include_directories(SYSTEM ${Boost_INCLUDE_DIR})
list(APPEND NEEDED_LIBRARIES ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY})

# Find Eigen
find_package(Eigen3 REQUIRED)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})
message(STATUS "Found Eigen3 at: " ${EIGEN3_INCLUDE_DIR})

# Find tbb
find_package(TBB COMPONENTS tbb tbbmalloc)
if(TBB_FOUND)
  list(APPEND NEEDED_LIBRARIES tbb tbbmalloc)
  message(STATUS "Found TBB")
elseif()
  message(STATUS "No tbb Found")
endif()

find_package(yaml-cpp REQUIRED)
include_directories(SYSTEM ${YAML_CPP_INCLUDE_DIR})

# Find GTSAM
find_package(GTSAM)
if(GTSAM_FOUND)
  # The following line might only work when gtsam is built on the same machine
  # get_target_property(GTSAM_LIBRARIES gtsam IMPORTED_LOCATION_RELEASE)
  #[[
  This line gets all of the include directories for gtsam as they have them
  spread out.
  ]]
  get_target_property(GTSAM_INCLUDE_DIR gtsam INTERFACE_INCLUDE_DIRECTORIES)
  # This set command works but I'm unsure how exactly
  set(GTSAM_LIBRARIES gtsam)
  message(STATUS "Found GTSAM:")
  message(STATUS "  ${GTSAM_LIBRARIES}")
  message(STATUS "Found GTSAM_INCLUDE_DIR:")
  message(STATUS "  ${GTSAM_INCLUDE_DIR}")
  include_directories(${GTSAM_INCLUDE_DIR})
  # Old way to find GTSAM library. Might still be useful in future
  # set(GTSAM_LIBRARIES "/usr/local/lib/libgtsam.so")
else()
  message(WARNING "No GTSAM Found. Most plugins and executables will not build.")
endif()


# ROS stuff
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        sensor_msgs
        geometry_msgs
        nav_msgs
        image_transport
        tf
        cmake_modules
)

# generate the messages
add_subdirectory(msgs)

# Add our source code
include_directories(${PROJECT_SOURCE_DIR}/include)

catkin_package (
        DEPENDS
        CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs sensor_msgs nav_msgs image_transport
        INCLUDE_DIRS include
)


include_directories(SYSTEM ${catkin_INCLUDE_DIRS})
include_directories(${CATKIN_DEVEL_PREFIX}/include)
message("What: " ${CATKIN_DEVEL_PREFIX}/include)

add_subdirectory(src)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        FILES_MATCHING PATTERN "*.launch" PATTERN "*.machine" PATTERN "*.yaml" PATTERN "*.urdf"
        )

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        )

