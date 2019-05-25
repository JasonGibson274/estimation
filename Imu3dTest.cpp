
/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file LocalizationExample.cp
 * @brief Simple robot localization example, with three "GPS-like" measurements
 * @author Jason Gibson and Bogdan Vlahov
 */

/**
 * A simple 2D pose slam example with "GPS" measurements
 *  - The robot moves forward 2 meter each iteration
 *  - The robot initially faces along the X axis (horizontal, to the right in 2D)
 *  - We have full odometry between pose
 *  - We have "GPS-like" measurements implemented with a custom factor
 */

// We will use Pose3 variables (x, y, z, r, p, y) to represent the robot pose
#include <gtsam/geometry/Pose3.h>

// Each variable in the system (poses and landmarks) must be identified with a
// unique key. We can either use simple integer keys (1, 2, 3, ...) or symbols
// (X1, X2, L1). Here we will use Symbols
#include <gtsam/inference/Symbol.h>

// As in OdometryExample.cpp, we use a BetweenFactor to model odometry measurements.
#include <gtsam/slam/BetweenFactor.h>

// We add all facors to a Nonlinear Factor Graph, as our factors are nonlinear.
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

// The nonlinear solvers within GTSAM are iterative solvers, meaning they linearize the
// nonlinear functions around an initial linearization point, then solve the linear system
// to update the linearization point. This happens repeatedly until the solver converges
// to a consistent set of variable values. This requires us to specify an initial guess
// for each variable, held in a Values container.
#include <gtsam/nonlinear/Values.h>

// we want isam to solve incrementally
#include <gtsam/nonlinear/ISAM2.h>

// In GTSAM, measurement functions are represented as 'factors'. Several common
// factors have been provided with the library for solving robotics/SLAM/Bundle
// Adjustment problems. Here we will use Projection factors to model the
// camera's landmark observations. Also, we will initialize the robot at some
// location using a Prior factor.
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>

// include the imu factors to constrain motion
// includes time varying bias between key frames
#include <gtsam/navigation/CombinedImuFactor.h>
// static bias
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/slam/PriorFactor.h>

using namespace std;
using namespace gtsam;

using symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)

#include <map>
#include <random>
#include <math.h>
int t = 0;
vector<double> true_position_ = {0, 0, 0};
vector<double> true_velocity_ = {0,0,0};
vector<double> estimated_position_ = {0, 0, 0};
vector<double> estimated_velocity_ = {0, 0, 0};

// Given an action, observation of the new state, and an estimate of the new state
// Recalculates the most likely states for all t and prints it out
void propogate_motion_graph(Pose3 new_pose, Vector3 new_vel) {
  t++;

}

int main(int argc, char** argv) {
  // create the prior on position and add it to the graph
  Point3 prior_position(0,0,5);
  Rot3 prior_rot = Rot3();
  Pose3 prior_pose(prior_rot, prior_position);
  Vector3 prior_velocity(0,0,0);

  // assume constant 0 bias
  imuBias::ConstantBias prior_imu_bias;

  // Assemble prior noise model and add it the graph.
  noiseModel::Diagonal::shared_ptr pose_noise_model = noiseModel::Diagonal::Sigmas((Vector(6) << 0.01, 0.01, 0.01, 0.5, 0.5, 0.5).finished()); // rad,rad,rad,m, m, m
  noiseModel::Diagonal::shared_ptr velocity_noise_model = noiseModel::Isotropic::Sigma(3,0.1); // m/s
  noiseModel::Diagonal::shared_ptr bias_noise_model = noiseModel::Isotropic::Sigma(6,1e-3);

  Values initial_values;
  initial_values.insert(X(t), prior_pose);
  initial_values.insert(V(t), prior_velocity);
  initial_values.insert(B(t), prior_imu_bias);

  // create the graph
  NonlinearFactorGraph graph;

  // Create (incremental) ISAM2 solver
  ISAM2 isam;

  // Add all prior factors (pose, velocity, bias) to the graph.
  graph.add(PriorFactor<Pose3>(X(t), prior_pose, pose_noise_model));
  graph.add(PriorFactor<Vector3>(V(t), prior_velocity,velocity_noise_model));
  graph.add(PriorFactor<imuBias::ConstantBias>(B(t), prior_imu_bias,bias_noise_model));

  // We use the sensor specs to build the noise model for the IMU factor.
  double accel_noise_sigma = 0.0003924;
  double gyro_noise_sigma = 0.000205689024915;
  double accel_bias_rw_sigma = 0.004905;
  double gyro_bias_rw_sigma = 0.000001454441043;
  Matrix33 measured_acc_cov = Matrix33::Identity(3,3) * pow(accel_noise_sigma,2);
  Matrix33 measured_omega_cov = Matrix33::Identity(3,3) * pow(gyro_noise_sigma,2);
  Matrix33 integration_error_cov = Matrix33::Identity(3,3)*1e-8; // error committed in integrating position from velocities
  Matrix33 bias_acc_cov = Matrix33::Identity(3,3) * pow(accel_bias_rw_sigma,2);
  Matrix33 bias_omega_cov = Matrix33::Identity(3,3) * pow(gyro_bias_rw_sigma,2);
  Matrix66 bias_acc_omega_int = Matrix::Identity(6,6)*1e-5; // error in the bias used for preintegration

  boost::shared_ptr<PreintegratedCombinedMeasurements::Params> p = PreintegratedCombinedMeasurements::Params::MakeSharedD(0.0);
  // PreintegrationBase params:
  p->accelerometerCovariance = measured_acc_cov; // acc white noise in continuous
  p->integrationCovariance = integration_error_cov; // integration uncertainty continuous
  // should be using 2nd order integration
  // PreintegratedRotation params:
  p->gyroscopeCovariance = measured_omega_cov; // gyro white noise in continuous
  // PreintegrationCombinedMeasurements params:
  p->biasAccCovariance = bias_acc_cov; // acc bias in continuous
  p->biasOmegaCovariance = bias_omega_cov; // gyro bias in continuous
  p->biasAccOmegaInt = bias_acc_omega_int;

  // use regular preintegrated
  PreintegratedImuMeasurements imu_preintegrated(p, prior_imu_bias);

  // Create Possible Actions to take
  std::map<std::string, std::vector<double>> possible_actions_;
  possible_actions_.insert(std::pair<std::string, std::vector<double>>("w", { 1, 0, 0, 0, 0, 0}));
  possible_actions_.insert(std::pair<std::string, std::vector<double>>("a", { 0, 1, 0, 0, 0, 0}));
  possible_actions_.insert(std::pair<std::string, std::vector<double>>("s", { -1, 0, 0, 0, 0, 0}));
  possible_actions_.insert(std::pair<std::string, std::vector<double>>("d", { 0, -1, 0, 0, 0, 0}));
  possible_actions_.insert(std::pair<std::string, std::vector<double>>("n", { 0, 0, 0, 0, 0, 0}));

  // SIM CONSTANTS

  // Random Gaussian Noise Generators for the action and observation
  std::default_random_engine generator;
  // std::vector<std::normal_distribution<double>> action_noise_;
  // action_noise_.push_back()
  std::normal_distribution<double> x_action_(0, 0.2);
  std::normal_distribution<double> y_action_(0, 0.2);
  std::normal_distribution<double> theta_action_(0, 0.1);

  std::normal_distribution<double> x_obs_(0, 0.1);
  std::normal_distribution<double> y_obs_(0, 0.1);

  // dt
  double dt = 1.0;

  Values initial_estimate;
  initial_estimate.insert(X(t), Pose3(Rot3(), Point3(estimated_position_[0], estimated_position_[1], estimated_position_[2])));
  initial_estimate.insert(V(t), Vector3(estimated_velocity_[0], estimated_velocity_[1], estimated_velocity_[2]));
  initial_estimate.insert(B(t), imuBias::ConstantBias());

  // While Loop Goes Here
  while(true) {
    // Assume key has been pressed
    std::cout << "\n\nPress a key:";
    std::string keypress = "w";
    std::cin >> keypress;
    if (keypress == "q") {
      break;
    }
    // Find action based on keypress
    auto action_it_ = possible_actions_.find(keypress);
    if (action_it_ == possible_actions_.end()) {
      std::cout << "Not a valid action. Press w, a, s, or d to move. Press q to quit." << std::endl;
    }else {
      // Get Action Noise
      double x_action_noise_ = x_action_(generator);
      double y_action_noise_ = y_action_(generator);
      double z_action_noise_ = theta_action_(generator);

      t += 1;

      vector<double> action_taken_t_ = action_it_->second;
      std::cout << "Time " << t << ":" << std::endl;
      std::cout << "Action: x = " << action_taken_t_[0] << ", y = " << action_taken_t_[1] << ", yaw rate = " << action_taken_t_[5] << std::endl;
      vector<double> noisy_action(3, 0), noisy_obs(2, 0), noisy_est(3, 0);
      // Update true State
      for (size_t i = 0; i < true_position_.size(); i++) {
        true_position_[i] += true_velocity_[i] + action_taken_t_[i] * 0.5 * pow(dt, 2);
        true_velocity_[i] += action_taken_t_[i] * dt;
      }
      std::cout << "True State: x = " << true_position_[0] << ", y = " << true_position_[1] << ", z = " << true_position_[2] << std::endl;
      noisy_action[0] = action_taken_t_[0] + x_action_noise_;
      noisy_action[1] = action_taken_t_[1] + y_action_noise_;
      noisy_action[2] = action_taken_t_[2] + z_action_noise_;
      for (size_t i = 0; i < estimated_position_.size(); i++) {
        estimated_position_[i] += estimated_velocity_[i] + noisy_action[i] * 0.5 * pow(dt, 2);
        estimated_velocity_[i] += noisy_action[i] * dt;
      }
      std::cout << "Noisy Action: x = " << noisy_action[0] << ", y = " << noisy_action[1] << ", theta = " << noisy_action[2] << std::endl;

      imu_preintegrated.integrateMeasurement(Vector3(noisy_action[0], noisy_action[1], noisy_action[2]), Vector3(0,0,0), dt);

      ImuFactor imu_factor(X(t-1), V(t-1),
                           X(t  ), V(t  ),
                           B(0),
                           imu_preintegrated);
      graph.add(imu_factor);
      imu_preintegrated.resetIntegration();

      initial_estimate.insert(X(t), Pose3(Rot3(), Point3(estimated_position_[0], estimated_position_[1], estimated_position_[2])));
      initial_estimate.insert(V(t), Vector3(estimated_velocity_[0], estimated_velocity_[1], estimated_velocity_[2]));

      isam.update(graph, initial_estimate);
      Values result = isam.calculateEstimate();
      graph = NonlinearFactorGraph();
      initial_estimate.clear();
      result.print("result after " + to_string(t) + " steps");
    }
  }
  return 0;
}