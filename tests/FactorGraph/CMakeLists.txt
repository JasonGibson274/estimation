if(BUILD_TESTS)
  set(TEST_CONFIG_FOLDER "${PROJECT_SOURCE_DIR}/config")
  # Create header file pointing to test config location
  set(TEST_CONFIG_HEADER_FILE ${PROJECT_BINARY_DIR}/include/autorally_factor_graph_test_config.h)
  configure_file(autorally_test_config.h.in ${TEST_CONFIG_HEADER_FILE})
  message(STATUS "Generated ${TEST_CONFIG_HEADER_FILE}")
  # Add the generated header file to list of includes
  include_directories(${PROJECT_BINARY_DIR}/include)
  message("")
endif()

set(PLANT_TARGET_NAME factor_graph_simple_tests)

#message(${GTSAM_LIBRARIES})
#message(${NEEDED_LIBRARIES})

set(GTEST_LIBRARIES gtest gmock gtest_main)
# set(TARGETS autorally_roboust_cost_test autorally_standard_cost_test
#     cartpole_quadratic_cost_test double_integrator_circle_cost_test
#     quadrotor_quadratic_cost_test)

file(GLOB TARGET_SRCS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cpp)

foreach(T_FILE IN LISTS TARGET_SRCS)
  get_filename_component(T_NAME ${T_FILE} NAME_WE)
  add_executable(${T_NAME} ../test_main.cpp ${T_FILE})
  target_link_libraries(${T_NAME}
          ${GTEST_LIBRARIES}
          FactorGraphEstimatorLib
          ${GTSAM_LIBRARIES}
          ${NEEDED_LIBRARIES}
          cnpy)
  #  add_test(NAME ${T_NAME} COMMAND ${T_NAME})
  gtest_discover_tests(${T_NAME})
  set_target_properties(${T_NAME} PROPERTIES FOLDER test)
endforeach()

