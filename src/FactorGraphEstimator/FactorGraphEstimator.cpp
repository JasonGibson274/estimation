#include <alphapilot/estimation/FactorGraph/FactorGraphEstimator.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>

using namespace std;
using namespace gtsam;

const double kGravity = 9.81;

namespace alphapilot {
namespace estimator {
  /**
   * Constructor for the Factor Graph Estimator;
   */
  FactorGraphEstimator::FactorGraphEstimator() {
    current_incremental_graph_ = make_shared<NonlinearFactorGraph>();
    // Construct parameters for preintegration
    auto params = PreintegrationParams::MakeSharedU(kGravity);
    params->setAccelerometerCovariance(I_3x3 * 0.1);
    params->setGyroscopeCovariance(I_3x3 * 0.1);
    params->setIntegrationCovariance(I_3x3 * 0.1);
    params->setUse2ndOrderCoriolis(false);
    params->setOmegaCoriolis(Vector3(0, 0, 0));
    preintegrator_imu_ = PreintegratedImuMeasurements(params);

    // Add initial bias and state to graph
    bias_index_ = Symbol('b', index);
    current_position_guess_ = make_shared<Pose3>(Rot3(), Point3(0, 0, 0));
    current_velocity_guess_ = make_shared<Vector3>(0, 0, 0);
    current_bias_guess_ = make_shared<imuBias::ConstantBias>();
    gtsam_current_state_initial_guess_->insert(symbol_shorthand::X(index),
                                              *current_position_guess_);
    gtsam_current_state_initial_guess_->insert(symbol_shorthand::V(index),
                                              *current_velocity_guess_);
    gtsam_current_state_initial_guess_->insert(bias_index_,
                                              *current_bias_guess_);

    // create priors on the state

    // Assemble prior noise model and add it the graph.
    noiseModel::Diagonal::shared_ptr pose_noise_model = noiseModel::Diagonal::Sigmas((Vector(6)
            << 0.01, 0.01, 0.01, 0.5, 0.5, 0.5).finished()); // rad,rad,rad,m, m, m
    noiseModel::Diagonal::shared_ptr velocity_noise_model = noiseModel::Isotropic::Sigma(3,0.1); // m/s
    noiseModel::Diagonal::shared_ptr bias_noise_model = noiseModel::Isotropic::Sigma(6,1e-3);

    current_incremental_graph_->add(PriorFactor<Pose3>(symbol_shorthand::X(index), *current_position_guess_, pose_noise_model));
    current_incremental_graph_->add(PriorFactor<Vector3>(symbol_shorthand::V(index), *current_velocity_guess_, velocity_noise_model));
    current_incremental_graph_->add(PriorFactor<imuBias::ConstantBias>(symbol_shorthand::B(index), *current_bias_guess_, bias_noise_model));

    // Construct parameters for ISAM optimizer
    ISAM2Params isam_parameters;
    isam_parameters.relinearizeThreshold = 0.01;
    isam_parameters.relinearizeSkip = 1;
    isam_parameters.cacheLinearizedFactors = false;
    isam_parameters.enableDetailedResults = true;
    isam_parameters.print();
    isam_ = make_shared<ISAM2>(isam_parameters);

    // Initialze K
    K_ = Cal3_S2(50.0, 50.0, 0.0, 50.0, 50.0);

    // Initialize Noise Models
    cam_measurement_noise_ =
      noiseModel::Isotropic::Sigma(2, 1.0);  // one pixel in u and v

    Vector6 covvec;
    covvec << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
    bias_noise_ = noiseModel::Diagonal::Variances(covvec);
  }

  /**
   * Creates a new state variable by increasing the state index
   * and adds it to the graph.
   * The basic process for doing so is creating a factor relating
   * previous state variables to current state variables (aka. motion model)
   * and then adding a guess for the current values of those states
   * to give the optimizer a value to start from
   */
  void FactorGraphEstimator::add_imu_factor() {
    // Update index to reflect time has passed
    index++;

    // Update bias at a slower rate
    if (index % 5 == 0) {
      bias_index_++;
      Symbol b1 = bias_index_ - 1;
      Symbol b2 = bias_index_;

      // Motion model of bias - currently constant bias is assumed

      // Add factor to graph and add initial variable guesses
      lock_guard<mutex> lck(graph_lck_);
      current_incremental_graph_->emplace_shared<BetweenFactor<
          imuBias::ConstantBias>>(b1, b2, imuBias::ConstantBias(),
                                  bias_noise_);
      gtsam_current_state_initial_guess_->insert(bias_index_,
                                                 *current_bias_guess_);
    }

    preintegrator_lck_.lock();
    // motion model of position and velocity variables
    ImuFactor imufac(symbol_shorthand::X(index - 1),
                     symbol_shorthand::V(index - 1),
                     symbol_shorthand::X(index),
                     symbol_shorthand::V(index),
                     bias_index_,
                     preintegrator_imu_);

    // clear IMU rolling integration
    preintegrator_imu_.resetIntegration();

    // Add factor to graph and add initial variable guesses
    lock_guard<mutex> lck(graph_lck_);
    gtsam_current_state_initial_guess_->insert(symbol_shorthand::X(index),
                                              *current_position_guess_);
    gtsam_current_state_initial_guess_->insert(symbol_shorthand::V(index),
                                              *current_velocity_guess_);
    // prevents current_vel and pos from being updated
    preintegrator_lck_.unlock();
    current_incremental_graph_->add(imufac);
  }

  /**
   * Takes in detected landmarks from image space and adds them to the factor
   *  graph using the structureless landmark factor from the following paper:
   *
   * C. Forster, L. Carlone, F. Dellaert, D. Scaramuzza, "IMU Preintegration on
   * Manifold for Efficient Visual-Inertial Maximum-a-Posteriori Estimation",
   * Robotics: Science and Systems (RSS), 2015.
   *
   * Optimizes the enitre state trajecotry at the end
   *
   * @param map<landmark_id, uv_coords> landmark_data
   */
  void FactorGraphEstimator::callback_cm(const map<int, pair<double, double>>
                                         landmark_data) {
    for (auto seen_landmark : landmark_data) {
      int l_id = seen_landmark.first;
      pair<double, double> im_coords = seen_landmark.second;
      auto landmark_factor_it = landmark_factors_.find(l_id);
      // New Landmark - Add a new Factor
      if (landmark_factor_it == landmark_factors_.end()) {
        // TODO magic number
        Cal3_S2::shared_ptr K(new Cal3_S2(50.0, 50.0, 0.0, 50.0, 50.0));
        landmark_factors_[l_id] =
          new SmartProjectionPoseFactor<Cal3_S2>(cam_measurement_noise_, K);
      }
      // Translate detection into gtsam
      Point2 detection_coords(im_coords.first, im_coords.second);
      // Create newest state
      add_imu_factor();
      // Add landmark to factor
      graph_lck_.lock();
      landmark_factors_[l_id]->add(detection_coords,
                                   symbol_shorthand::X(index));
      graph_lck_.unlock();

      run_optimize();
    }
  }

  void FactorGraphEstimator::callback_imu(IMU_readings imu_data) {
    // integrate the imu reading
  }

  /**
   * Adds the local factor graph that was constructed during the most recent
   * callbacks, then clears the graph for the next iteration.
   * This optimizes the entire state trajectory
   */
  void FactorGraphEstimator::run_optimize() {
    // run the optimization and output the final state
  }
} // estimator
} // alphapilot