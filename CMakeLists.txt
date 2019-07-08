cmake_minimum_required(VERSION 3.00)
project(StateEstimator)

# Add custom cmake finds
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")

# Add macro for subdirectories
include(MacroSubDirList)
# Fix needed for linaro gcc compiler according to https://gcc.gnu.org/onlinedocs/gcc/Directory-Options.html
#set(CMAKE_INCLUDE_SYSTEM_FLAG_CXX "-I")
message("WE ARE USING ${CMAKE_INCLUDE_SYSTEM_FLAG_CXX}")

option(ESTIMATION_BUILD_EXAMPLES_ALWAYS "Whether or not executables (tests) should be built" ON)
set(BUILD_FLAGS "-std=c++11 -Wuninitialized -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${BUILD_FLAGS}")

# Allow the executable location to be overwritten from command line
if (NOT CMAKE_RUNTIME_OUTPUT_DIRECTORY)
	set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)
endif()

# Allow the lib location to be overwritten from command line
if (NOT CMAKE_LIBRARY_OUTPUT_DIRECTORY)
	get_filename_component(PROJECT_LIBS_DIR ${PROJECT_BINARY_DIR}/lib ABSOLUTE)
	set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
	message("${PROJECT_LIBS_DIR}")
else()
	get_filename_component(PROJECT_LIBS_DIR ${CMAKE_LIBRARY_OUTPUT_DIRECTORY} ABSOLUTE)
endif()

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


# Find tbb
find_package(TBB COMPONENTS tbb tbbmalloc)
if(TBB_FOUND)
	list(APPEND NEEDED_LIBRARIES tbb tbbmalloc)
    message(STATUS "Found TBB")
elseif()
	message(STATUS "No tbb Found")
endif()

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

find_package(yaml-cpp)
if (yaml-cpp_FOUND)
	include_directories(${YAML_CPP_INCLUDE_DIR})
	add_definitions(-DENABLE_YAML=1)
endif ()

# Add our source code
include_directories(${PROJECT_SOURCE_DIR}/include)

add_subdirectory(src)
if (ESTIMATION_BUILD_EXAMPLES_ALWAYS)
	add_subdirectory(main)
endif()

foreach(state_plugin ${PROJECT_PLUGINS})
	message(STATUS "Building ${state_plugin}...")
endforeach()

# Install all library header files
install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
  )

# Install alpha pilot common
install(
  DIRECTORY include/alphapilot_common
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
  )

set(${PROJECT_NAME}_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include PARENT)
# TODO set up properly lower
set(${PROJECT_NAME}_LIBRARIES FactorGraphEstimator_plugin PARENT)
