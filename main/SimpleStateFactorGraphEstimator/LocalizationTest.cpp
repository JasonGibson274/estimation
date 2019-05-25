
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
 * @author Frank Dellaert
 */

/**
 * A simple 2D pose slam example with "GPS" measurements
 *  - The robot moves forward 2 meter each iteration
 *  - The robot initially faces along the X axis (horizontal, to the right in 2D)
 *  - We have full odometry between pose
 *  - We have "GPS-like" measurements implemented with a custom factor
 */

// We will use Pose2 variables (x, y, theta) to represent the robot positions
#include <gtsam/geometry/Pose2.h>

#include <gtsam/geometry/Pose3.h>

// We will use simple integer Keys to refer to the robot poses.
#include <gtsam/inference/Key.h>

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

// We want to use iSAM2 to solve the structure-from-motion problem
// incrementally, so include iSAM2 here
#include <gtsam/nonlinear/ISAM2.h>

// Finally, once all of the factors have been added to our factor graph, we will want to
// solve/optimize to graph to find the best (Maximum A Posteriori) set of variable values.
// GTSAM includes several nonlinear optimizers to perform this step. Here we will use the
// standard Levenberg-Marquardt solver
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

// Once the optimized values have been calculated, we can also calculate the marginal covariance
// of desired variables
#include <gtsam/nonlinear/Marginals.h>

using namespace std;
using namespace gtsam;

// Before we begin the example, we must create a custom unary factor to implement a
// "GPS-like" functionality. Because standard GPS measurements provide information
// only on the position, and not on the orientation, we cannot use a simple prior to
// properly model this measurement.
//
// The factor will be a unary factor, affect only a single system variable. It will
// also use a standard Gaussian noise model. Hence, we will derive our new factor from
// the NoiseModelFactor1.
#include <gtsam/nonlinear/NonlinearFactor.h>

class UnaryFactor: public NoiseModelFactor1<Pose2> {

    // The factor will hold a measurement consisting of an (X,Y) location
    // We could this with a Point2 but here we just use two doubles
    double mx_, my_;

public:
    /// shorthand for a smart pointer to a factor
    typedef boost::shared_ptr<UnaryFactor> shared_ptr;

    // The constructor requires the variable key, the (X, Y) measurement value, and the noise model
    UnaryFactor(Key j, double x, double y, const SharedNoiseModel& model):
            NoiseModelFactor1<Pose2>(model, j), mx_(x), my_(y) {}

    virtual ~UnaryFactor() {}

    // Using the NoiseModelFactor1 base class there are two functions that must be overridden.
    // The first is the 'evaluateError' function. This function implements the desired measurement
    // function, returning a vector of errors when evaluated at the provided variable value. It
    // must also calculate the Jacobians for this measurement function, if requested.
    Vector evaluateError(const Pose2& q, boost::optional<Matrix&> H = boost::none) const
    {
        // The measurement function for a GPS-like measurement is simple:
        // error_x = pose.x - measurement.x
        // error_y = pose.y - measurement.y
        // Consequently, the Jacobians are:
        // [ derror_x/dx  derror_x/dy  derror_x/dtheta ] = [1 0 0]
        // [ derror_y/dx  derror_y/dy  derror_y/dtheta ] = [0 1 0]
        if (H) (*H) = (Matrix(2,3) << 1.0,0.0,0.0, 0.0,1.0,0.0).finished();
        return (Vector(2) << q.x() - mx_, q.y() - my_).finished();
    }

    // The second is a 'clone' function that allows the factor to be copied. Under most
    // circumstances, the following code that employs the default copy constructor should
    // work fine.
    virtual gtsam::NonlinearFactor::shared_ptr clone() const {
        return boost::static_pointer_cast<gtsam::NonlinearFactor>(
                gtsam::NonlinearFactor::shared_ptr(new UnaryFactor(*this))); }

    // Additionally, we encourage you the use of unit testing your custom factors,
    // (as all GTSAM factors are), in which you would need an equals and print, to satisfy the
    // GTSAM_CONCEPT_TESTABLE_INST(T) defined in Testable.h, but these are not needed below.

}; // UnaryFactor
#include <map>
#include <random>
#include <chrono>
typedef std::chrono::high_resolution_clock Clock;
typedef std::chrono::milliseconds milliseconds;
// Create graph and state structures
NonlinearFactorGraph graph;
int t = 0;
vector<Pose2> state_estimates;
vector<Pose3> landmarks;
vector<Point2> camera_measurements;
// Values initialStateEstimate;
ISAM2 isam;
vector<double> true_state_(3, 0); // Initialize state at origin
map<pair<double, double>, vector<int>> previous_states;

// Given an action, observation of the new state, and an estimate of the new state
// Recalculates the most likely states for all t and prints it out
void step_graph(Pose2 action, std::vector<double> obs, Pose2 estimate_t) {
  // Motion Model Update
  auto start_time = Clock::now();
  noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1));
  graph.emplace_shared<BetweenFactor<Pose2> >(t, t+1, action, odometryNoise);
  t++;

  // Sensor Update
  noiseModel::Diagonal::shared_ptr unaryNoise = noiseModel::Diagonal::Sigmas(Vector2(0.1, 0.1)); // 10cm std on x,y
  graph.emplace_shared<UnaryFactor>(t, obs[0], obs[1], unaryNoise);
  cout << "Added factors at this state" << endl;
  // Check for Loop Closures
  auto it_ = previous_states.find(std::make_pair(true_state_[0], true_state_[1]));
  if (it_ == previous_states.end()) {
    previous_states.insert(std::pair<std::pair<double, double>, vector<int>>(std::make_pair(true_state_[0], true_state_[1]), {t}));
  } else {
    auto previous_times = it_->second;
    for (auto t_ : previous_times) {
      graph.emplace_shared<BetweenFactor<Pose2> >(t_, t, Pose2(0, 0, 0), odometryNoise);
      std::cout << "Loop Closed between " << t_ << " and " << t << std::endl;
    }
    it_->second.push_back(t);

  }

  // Update state estimates for all time
  Values initialStateEstimate;
  state_estimates.push_back(estimate_t);
  initialStateEstimate.insert(t, state_estimates[t]);
  if (t == 1) {
    initialStateEstimate.insert(0, state_estimates[0]);
  }
  // for (size_t i = 0; i < state_estimates.size(); i++) {
  //   initialStateEstimate.insert(i, state_estimates[i]);
  // }
  // Optimize

  // LevenbergMarquardtOptimizer optimizer(graph, initialStateEstimate);
  // Values result = optimizer.optimize();
  // result.print("Final Result:\n");

  isam.update(graph, initialStateEstimate);
  isam.update();
  Values result = isam.calculateEstimate();
  auto end_time = Clock::now();
  auto calc_time = std::chrono::duration_cast<milliseconds>(end_time - start_time);
  result.print("Final Result after optimazation:\n");
  // TODO: Update state estimate with values from result
  cout << "Took " << calc_time.count() << " ms" << endl;
  graph = NonlinearFactorGraph();
  // Marginals marginals(graph, result);

  // cout << "Latest state covariance:\n" << marginals.marginalCovariance(t) << endl;

}

int main(int argc, char** argv) {
  // // 2b. Add "GPS-like" measurements
  // // We will use our custom UnaryFactor for this.

  ISAM2Params parameters;
  parameters.relinearizeThreshold = 0.01;
  parameters.relinearizeSkip = 1;
  isam = ISAM2(parameters);
  noiseModel::Diagonal::shared_ptr unaryNoise = noiseModel::Diagonal::Sigmas(Vector2(0.1, 0.1)); // 10cm std on x,y

  // Give iniitial observation as perfect reading of the true_state
  graph.emplace_shared<UnaryFactor>(t, true_state_[0], true_state_[1], unaryNoise);
  // Initial estimate of state is the origin
  state_estimates.push_back(Pose2(true_state_[0], true_state_[1], true_state_[2]));
  previous_states.insert(std::pair<std::pair<double, double>, vector<int>>(std::make_pair(true_state_[0], true_state_[1]), {t}));

  // Create Possible Actions to take
  std::map<std::string, std::vector<double>> possible_actions_;
  possible_actions_.insert(std::pair<std::string, std::vector<double>>("w", { 0, 10, 0}));
  possible_actions_.insert(std::pair<std::string, std::vector<double>>("a", {-10, 0, 0}));
  possible_actions_.insert(std::pair<std::string, std::vector<double>>("s", { 0,-10, 0}));
  possible_actions_.insert(std::pair<std::string, std::vector<double>>("d", { 10, 0, 0}));

  // Random Gaussian Noise Generators for the action and observation
  std::default_random_engine generator;
  // std::vector<std::normal_distribution<double>> action_noise_;
  // action_noise_.push_back()
  std::normal_distribution<double> x_action_(0, 0.2);
  std::normal_distribution<double> y_action_(0, 0.2);
  std::normal_distribution<double> theta_action_(0, 0.1);

  std::normal_distribution<double> x_obs_(0, 0.1);
  std::normal_distribution<double> y_obs_(0, 0.1);

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
      double theta_action_noise_ = theta_action_(generator);
      // Get Observation Noise
      double x_obs_noise_ = x_obs_(generator);
      double y_obs_noise_ = y_obs_(generator);

      vector<double> action_taken_t_ = action_it_->second;
      std::cout << "Time " << t+1 << ":" << std::endl;
      std::cout << "Action: x = " << action_taken_t_[0] << ", y = " << action_taken_t_[1] << ", theta = " << action_taken_t_[2] << std::endl;
      vector<double> noisy_action(3, 0), noisy_obs(2, 0), noisy_est(3, 0);
      // Update true State
      for (size_t i = 0; i < true_state_.size(); i++) {
        true_state_[i] += action_taken_t_[i];
      }
      std::cout << "True State: x = " << true_state_[0] << ", y = " << true_state_[1] << ", theta = " << true_state_[2] << std::endl;
      noisy_action[0] = action_taken_t_[0] + x_action_noise_;
      noisy_action[1] = action_taken_t_[1] + y_action_noise_;
      noisy_action[2] = action_taken_t_[2] + theta_action_noise_;
      std::cout << "Noisy Action: x = " << noisy_action[0] << ", y = " << noisy_action[1] << ", theta = " << noisy_action[2] << std::endl;

      noisy_obs[0] = true_state_[0] + x_obs_noise_;
      noisy_obs[1] = true_state_[1] + y_obs_noise_;
      std::cout << "Noisy Obs: x = " << noisy_obs[0] << ", y = " << noisy_obs[1] << std::endl;

      // for (int i = 0; i < true_state_.size(); i++) {
      //   noisy_est[i] = true_state_[i] + noisy_action[i];
      // }
      noisy_est[0] = state_estimates.back().x() + noisy_action[0];
      noisy_est[1] = state_estimates.back().y() + noisy_action[1];
      noisy_est[2] = state_estimates.back().theta() + noisy_action[2];

      step_graph(Pose2(noisy_action[0], noisy_action[1], noisy_action[2]), noisy_obs, Pose2(noisy_est[0],noisy_est[1], noisy_est[2]));
    }
  }
  return 0;
}
