#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <random>
#include <algorithm>
#include <numeric>

#include <StateEstimator/FactorGraph/FactorGraphEstimator.h>
#include <autorally_factor_graph_test_config.h>

TEST(FactorGraphEstimatorLibSimple, Constructor) {
  estimator::FactorGraphEstimator estimator(tests::test_config_file);
}

