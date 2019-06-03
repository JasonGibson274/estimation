#include <StateEstimator/estimation/FactorGraph/FactorGraphEstimator.h>
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
  FactorGraphEstimator::FactorGraphEstimator(const std::shared_ptr<drone_state>& initial_state) {
    gtsam_current_state_initial_guess_ = make_shared<Values>();
    K_ = boost::make_shared<gtsam::Cal3_S2>(548.4088134765625, 548.4088134765625, 0, 512.0, 384.0);
    boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> p = PreintegratedCombinedMeasurements::Params::MakeSharedD(0.0);
    // TODO params
    // PreintegrationBase params:
    //p->accelerometerCovariance = measured_acc_cov; // acc white noise in continuous
    //p->integrationCovariance = integration_error_cov; // integration uncertainty continuous
    // should be using 2nd order integration
    // PreintegratedRotation params:
    //p->gyroscopeCovariance = measured_omega_cov; // gyro white noise in continuous
    // PreintegrationCombinedMeasurements params:
    //p->biasAccCovariance = bias_acc_cov; // acc bias in continuous
    //p->biasOmegaCovariance = bias_omega_cov; // gyro bias in continuous
    //p->biasAccOmegaInt = bias_acc_omega_int;

    // use regular preintegrated
    preintegrator_imu_ = PreintegratedImuMeasurements(p, gtsam::imuBias::ConstantBias());
    resetGraph(initial_state);

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
      Symbol b1 = symbol_shorthand::B(bias_index_ - 1);
      Symbol b2 = symbol_shorthand::B(bias_index_);

      // Motion model of bias - currently constant bias is assumed

      // Add factor to graph and add initial variable guesses
      lock_guard<mutex> lck(graph_lck_);
      current_incremental_graph_->emplace_shared<BetweenFactor<
          imuBias::ConstantBias>>(b1, b2, imuBias::ConstantBias(),
                                  bias_noise_);
      gtsam_current_state_initial_guess_->insert(symbol_shorthand::B(bias_index_),
                                                 *current_bias_guess_);
    }

    preintegrator_lck_.lock();
    // motion model of position and velocity variables
    ImuFactor imufac(symbol_shorthand::X(index - 1),
                     symbol_shorthand::V(index - 1),
                     symbol_shorthand::X(index),
                     symbol_shorthand::V(index),
                     symbol_shorthand::B(bias_index_),
                     preintegrator_imu_);

    // clear IMU rolling integration
    preintegrator_imu_.resetIntegration();
    imu_meas_count_ = 0;

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
  void FactorGraphEstimator::callback_cm(const std::shared_ptr<map<std::string, pair<double, double>>>
                                         landmark_data) {
    // TODO verify that the landmark data is in the camera FOV
    // if there is no IMU's then this is unconstrained
    if(imu_meas_count_ <= 0) {
      return;
    }
    // Create newest state
    add_imu_factor();
    for (const auto& seen_landmark : *landmark_data) {
      std::cout << "adding landmark detection " << seen_landmark.first << ", " << seen_landmark.second.first << ", " << seen_landmark.second.second << std::endl;
      std::string l_id = seen_landmark.first;
      pair<double, double> im_coords = seen_landmark.second;
      auto landmark_factor_it = landmark_factors_.find(l_id);

      // New Landmark - Add a new Factor
      if (landmark_factor_it == landmark_factors_.end()) {
        landmark_factors_[l_id] =
          new SmartProjectionPoseFactor<Cal3_S2>(cam_measurement_noise_, K_);
      }
      // Translate detection into gtsam
      Point2 detection_coords(im_coords.first, im_coords.second);

      // Add landmark to factor
      graph_lck_.lock();
      landmark_factors_[l_id]->add(detection_coords,
                                   symbol_shorthand::X(index));
      graph_lck_.unlock();

    }

    run_optimize();
  }

  void FactorGraphEstimator::callback_imu(std::shared_ptr<IMU_readings> imu_data) {
    // integrate the imu reading using affine dynamics from TODO cite
    // updates the current guesses of position
    Vector3 accel = Vector3 (imu_data->x_accel, imu_data->y_accel, imu_data->z_accel);
    // TODO check order
    Vector3 ang_rate = Vector3 (imu_data->roll_vel, imu_data->pitch_vel, imu_data->yaw_vel);
    lock_guard<mutex> preintegration_lck(preintegrator_lck_);
    preintegrator_imu_.integrateMeasurement(accel, ang_rate, imu_data->dt);
    imu_meas_count_++;
    std::cout << "integrate IMU\n" << accel << "\n" << ang_rate << std::endl;

    // integrate the IMU to get an updated estimate of the current position
    // TODO does this make sense?
    propagate_imu(current_pose_estimate_, accel, ang_rate, imu_data->dt);
  }

  /**
   * Adds the local factor graph that was constructed during the most recent
   * callbacks, then clears the graph for the next iteration.
   * This optimizes the entire state trajectory
   */
  void FactorGraphEstimator::run_optimize() {
    std::cout << "optimize" << std::endl;
    // run the optimization and output the final state
    lock_guard<mutex> graph_lck(graph_lck_);
    lock_guard<mutex> preintegration_lck(preintegrator_lck_);

    //gtsam_current_state_initial_guess_->print();

    // TODO is bias guess being updated?
    //gtsam_current_state_initial_guess_->insert(symbol_shorthand::B(index), current_bias_guess_);

    // create copy of the graph just in case
    NonlinearFactorGraph isam_graph = current_incremental_graph_->clone();
    isam_graph.print();
    gtsam_current_state_initial_guess_->print();
    // run update and run optimization
    isam_->update(isam_graph, *gtsam_current_state_initial_guess_);
    // set the guesses of state to the correct output
    current_position_guess_ = make_shared<Pose3>(isam_->calculateEstimate<Pose3>(symbol_shorthand::X(index)));
    current_velocity_guess_ = make_shared<Vector3>(isam_->calculateEstimate<Vector3>(symbol_shorthand::V(index)));
    current_bias_guess_ = make_shared<imuBias::ConstantBias>(isam_->calculateEstimate<imuBias::ConstantBias>(symbol_shorthand::B(bias_index_)));

    // set the result to best guess
    current_pose_estimate_.x = current_position_guess_->x();
    current_pose_estimate_.y = current_position_guess_->y();
    current_pose_estimate_.z = current_position_guess_->z();

    current_pose_estimate_.roll = current_position_guess_->rotation().rpy()[0];
    current_pose_estimate_.pitch = current_position_guess_->rotation().rpy()[1];
    current_pose_estimate_.yaw = current_position_guess_->rotation().rpy()[2];

    current_pose_estimate_.x_dot = (*current_velocity_guess_)[0];
    current_pose_estimate_.y_dot = (*current_velocity_guess_)[1];
    current_pose_estimate_.z_dot = (*current_velocity_guess_)[2];

    // clear incremental graph and current guesses
    current_incremental_graph_ = make_shared<NonlinearFactorGraph>();
    gtsam_current_state_initial_guess_->clear();

  }

  /**
   * Propagates the state according to the (stochastic) rigid body dynamics equations
   * Modifies class members
   * @param acc linear acceleration in body frame
   * @param angular_vel in body frame
   * @param dt time difference
   */
   // TODO clean up method
  void FactorGraphEstimator::propagate_imu(drone_state current_state, Vector3 acc, Vector3 angular_vel, double dt) {
    drone_state result;

    //TODO this is wrong
    current_state.x = current_position_guess_->x();
    current_state.y = current_position_guess_->y();
    current_state.z = current_position_guess_->z();

    current_state.x_dot = (*current_velocity_guess_)[0];
    current_state.y_dot = (*current_velocity_guess_)[1];
    current_state.z_dot = (*current_velocity_guess_)[2];

    current_state.roll = current_position_guess_->rotation().rpy()[0];
    current_state.pitch = current_position_guess_->rotation().rpy()[1];
    current_state.yaw = current_position_guess_->rotation().rpy()[2];

    //Update angular rate
    result.roll_dot = angular_vel[0];
    result.pitch_dot = angular_vel[1];
    result.pitch_dot = angular_vel[2];

    //Position updates
    result.x = current_state.x + current_state.x_dot*dt;
    result.y = current_state.y + current_state.y_dot*dt;
    result.z = current_state.z + current_state.z_dot*dt;

    //Update velocity
    float c_phi, c_theta, c_psi, s_phi, s_theta, s_psi;
    float ux, uy, uz;
    c_phi = cosf(current_state.roll);
    c_theta = cosf(current_state.pitch);
    c_psi = cosf(current_state.yaw);
    s_phi = sinf(current_state.roll);
    s_theta = sinf(current_state.pitch);
    s_psi = sinf(current_state.yaw);
    ux = acc[0]*dt;
    uy = acc[1]*dt;
    uz = acc[2]*dt;
    result.x_dot = current_state.x_dot + (c_theta*c_psi)*ux + (s_phi*s_theta*c_psi - c_phi*s_psi)*uy + (c_phi*s_theta*c_psi + s_phi*s_psi)*uz;
    result.y_dot = current_state.y_dot + (c_theta*s_psi)*ux + (s_phi*s_theta*s_psi + c_phi*c_psi)*uy + (c_phi*s_theta*s_psi - s_phi*c_psi)*uz;
    result.z_dot = current_state.z_dot + (-s_theta)*ux + (c_theta*s_phi)*uy + (c_theta*c_phi)*uz - 9.81*dt;

    //Update the euler angles
    float r, p, y;
    r = current_state.roll + (current_state.roll_dot + (s_phi*s_theta/c_theta)*current_state.pitch_dot + (c_phi*s_theta/c_theta)*current_state.yaw_dot)*dt;
    p = current_state.pitch + (c_phi*current_state.pitch_dot - s_phi*current_state.yaw_dot)*dt;
    y = current_state.yaw + (s_phi/c_theta*current_state.pitch_dot + c_phi/c_theta*current_state.yaw_dot)*dt;
    result.roll = r;
    result.pitch = p;
    result.yaw = y;



    std::cout << "x = " << result.x << std::endl;

    // apply the update
    // the ordering is correct for gtsam
    Pose3 pose_result = Pose3(Rot3::Ypr(result.yaw, result.pitch, result.roll),
            Point3(result.x, result.y, result.z));
    *current_position_guess_ = *current_position_guess_ * pose_result;
    Vector3 vel_result = Vector3(result.x_dot, result.y_dot, result.z_dot);
    *current_velocity_guess_ = *current_velocity_guess_ + vel_result;

  }

  void FactorGraphEstimator::callback_range(int rangestuff) {
  }

  void FactorGraphEstimator::resetGraph(const std::shared_ptr<drone_state> initial_state) {
    lock_guard<mutex> graph_lck(graph_lck_);
    lock_guard<mutex> preintegration_lck(preintegrator_lck_);

    //clear
    index = 0;
    bias_index_ = 0;
    current_incremental_graph_ = make_shared<NonlinearFactorGraph>();
    preintegrator_imu_.resetIntegration();
    current_position_guess_ = make_shared<Pose3>(Rot3::Ypr(initial_state->yaw, initial_state->pitch, initial_state->roll),
            Point3(initial_state->x,initial_state->y,initial_state->z));
    current_velocity_guess_ = make_shared<Vector3>(initial_state->x_dot, initial_state->y_dot, initial_state->z_dot);
    current_bias_guess_ = make_shared<imuBias::ConstantBias>();
    gtsam_current_state_initial_guess_->clear();

    // insert initial guesses of state
    gtsam_current_state_initial_guess_->insert(symbol_shorthand::X(index),
                                              *current_position_guess_);
    gtsam_current_state_initial_guess_->insert(symbol_shorthand::V(index),
                                              *current_velocity_guess_);
    gtsam_current_state_initial_guess_->insert(symbol_shorthand::B(bias_index_),
                                              *current_bias_guess_);

    //gtsam_current_state_initial_guess_->print();

    // create priors on the state

    // Assemble prior noise model and add it the graph.
    noiseModel::Diagonal::shared_ptr pose_noise_model = noiseModel::Diagonal::Sigmas((Vector(6)
            << 0.01, 0.01, 0.01, 0.5, 0.5, 0.5).finished()); // rad,rad,rad,m, m, m
    noiseModel::Diagonal::shared_ptr velocity_noise_model = noiseModel::Isotropic::Sigma(3,0.1); // m/s
    noiseModel::Diagonal::shared_ptr bias_noise_model = noiseModel::Isotropic::Sigma(6,1e-3);

    // priors match initial guess
    current_incremental_graph_->add(PriorFactor<Pose3>(symbol_shorthand::X(index), *current_position_guess_, pose_noise_model));
    current_incremental_graph_->add(PriorFactor<Vector3>(symbol_shorthand::V(index), *current_velocity_guess_, velocity_noise_model));
    current_incremental_graph_->add(PriorFactor<imuBias::ConstantBias>(symbol_shorthand::B(bias_index_), *current_bias_guess_, bias_noise_model));

    // Construct parameters for ISAM optimizer
    ISAM2Params isam_parameters;
    isam_parameters.relinearizeThreshold = 0.01;
    isam_parameters.relinearizeSkip = 1;
    isam_parameters.cacheLinearizedFactors = false;
    isam_parameters.enableDetailedResults = true;
    isam_parameters.print();
    isam_ = make_shared<ISAM2>(isam_parameters);
  }

  drone_state FactorGraphEstimator::latest_state() {
    lock_guard<mutex> graph_lck(graph_lck_);
    return current_pose_estimate_;
  }

} // estimator
} // StateEstimator