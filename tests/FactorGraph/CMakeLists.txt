# create the
if(BUILD_TESTS)
  set(TEST_CONFIG_FOLDER "${PROJECT_SOURCE_DIR}/config")
  # Create header file pointing to test config location
  set(TEST_CONFIG_HEADER_FILE ${PROJECT_BINARY_DIR}/include/autorally_factor_graph_test_config.h)
  configure_file(autorally_test_config.h.in ${TEST_CONFIG_HEADER_FILE})
  message(STATUS "Generated ${TEST_CONFIG_HEADER_FILE}")
  # Add the generated header file to list of includes
  include_directories(${PROJECT_BINARY_DIR}/include)
endif()

set(PLANT_TARGET_NAME factor_graph_simple_tests)

add_executable(factor_graph_simple_test
        ../test_main.cpp
        FactorGraphSimpleTests.cpp)

target_link_libraries(factor_graph_simple_test gtest
        gmock
        gtest_main
        FactorGraphEstimatorLib
        ${GTSAM_LIBRARIES}
        ${NEEDED_LIBRARIES})

gtest_discover_tests(factor_graph_simple_test)
set_target_properties(factor_graph_simple_test PROPERTIES FOLDER test)

