#include <StateEstimator/estimation/FactorGraph/FactorGraphEstimator.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>

#include <yaml-cpp/yaml.h>

using namespace std;
using namespace gtsam;

const double kGravity = 9.81;

namespace alphapilot {
namespace estimator {
/**
 * Constructor for the Factor Graph Estimator; sets up the noise models
 * @param initial_state, start location to begin the optimization
 */
FactorGraphEstimator::FactorGraphEstimator(const estimator_config &estimator_config) {
  // TODO not supported start anymore
  debug_ = estimator_config.debug;
  last_imu_time_ = estimator_config.time;
  prior_config_ = estimator_config.priorConfig;

  // Construct parameters for ISAM optimizer
  isam_parameters_.relinearizeThreshold = estimator_config.isamParameters.relinearizeThreshold;
  isam_parameters_.relinearizeSkip = estimator_config.isamParameters.relinearizeSkip;
  isam_parameters_.enablePartialRelinearizationCheck =
      estimator_config.isamParameters.enablePartialRelinearizationCheck;
  isam_parameters_.cacheLinearizedFactors = estimator_config.isamParameters.chacheLinearedFactors;
  isam_parameters_.enableDetailedResults = estimator_config.isamParameters.enableDetailedResults;
  isam_parameters_.findUnusedFactorSlots = estimator_config.isamParameters.findUnusedFactorSlots;
  gtsam::ISAM2GaussNewtonParams optimizationParams;
  optimizationParams.wildfireThreshold = estimator_config.isamParameters.gaussianWildfireThreshold;
  isam_parameters_.optimizationParams = optimizationParams;
  isam_ = ISAM2(isam_parameters_);

  // camera
  use_camera_factors_ = estimator_config.cameraFactorParams.useCameraFactor;

  // IMU
  use_imu_factors_ = estimator_config.imuFactorParams.useImuFactor;
  invert_x_ = estimator_config.imuFactorParams.invertX;
  invert_y_ = estimator_config.imuFactorParams.invertY;
  invert_z_ = estimator_config.imuFactorParams.invertZ;
  Vector6 covvec;
  std::vector<double> covvec_arr = estimator_config.imuFactorParams.biasNoise;
  covvec << covvec_arr[0], covvec_arr[1], covvec_arr[2], covvec_arr[3], covvec_arr[4], covvec_arr[5];
  bias_noise_ = noiseModel::Diagonal::Variances(covvec);

  // setup IMU preintegrator parameters
  boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params>
      p = PreintegratedCombinedMeasurements::Params::MakeSharedU(GRAVITY);
  double accel_noise_sigma = estimator_config.imuFactorParams.accelNoiseSigma;
  double gyro_noise_sigma = estimator_config.imuFactorParams.gyroNoiseSigma;
  double accel_bias_rw_sigma = estimator_config.imuFactorParams.accelBiasRwSigma;
  double gyro_bias_rw_sigma = estimator_config.imuFactorParams.gyroBiasRwSigma;
  Matrix33 measured_acc_cov = Matrix33::Identity(3, 3) * pow(accel_noise_sigma, 2);
  Matrix33 measured_omega_cov = Matrix33::Identity(3, 3) * pow(gyro_noise_sigma, 2);
  // error committed in integrating position from velocities
  Matrix33 integration_error_cov = Matrix33::Identity(3, 3) * estimator_config.imuFactorParams.integrationErrorCov;
  Matrix33 bias_acc_cov = Matrix33::Identity(3, 3) * pow(accel_bias_rw_sigma, 2);
  Matrix33 bias_omega_cov = Matrix33::Identity(3, 3) * pow(gyro_bias_rw_sigma, 2);
  // error in the bias used for preintegration
  Matrix66 bias_acc_omega_int = Matrix::Identity(6, 6) * estimator_config.imuFactorParams.biasAccOmegaInt;

  // add them to the params
  p->accelerometerCovariance = measured_acc_cov; // acc white noise in continuous
  p->integrationCovariance = integration_error_cov; // integration uncertainty continuous
  p->gyroscopeCovariance = measured_omega_cov; // gyro white noise in continuous
  p->biasAccCovariance = bias_acc_cov; // acc bias in continuous
  p->biasOmegaCovariance = bias_omega_cov; // gyro bias in continuous
  p->biasAccOmegaInt = bias_acc_omega_int;

  // use regular preintegrated
  preintegrator_imu_ = PreintegratedImuMeasurements(p, gtsam::imuBias::ConstantBias());

  // pose factor noise
  use_pose_factors_ = estimator_config.poseFactorParams.usePoseFactor;
  std::vector<double> odom_vel_arr = estimator_config.poseFactorParams.poseVelNoise;
  Vector3 odom_vel_noise = Vector3(odom_vel_arr[0], odom_vel_arr[1], odom_vel_arr[2]);
  std::cout << "odom vel noise = " << odom_vel_noise << std::endl;
  odometry_vel_noise_ = noiseModel::Diagonal::Sigmas(odom_vel_noise);
  Vector6 pose_noise;
  std::vector<double> pose_noise_arr = estimator_config.poseFactorParams.poseNoise;
  pose_noise
      << pose_noise_arr[0], pose_noise_arr[1], pose_noise_arr[2], pose_noise_arr[3], pose_noise_arr[4], pose_noise_arr[5];
  std::cout << "pose noise = " << pose_noise << std::endl;
  odometry_pose_noise_ = noiseModel::Diagonal::Sigmas(pose_noise);

  // add the priors on state
  add_priors(estimator_config.priorConfig.initial_state);
}

FactorGraphEstimator::FactorGraphEstimator(const std::string &config_file) {
  YAML::Node config = YAML::LoadFile(config_file);

  if (config["factorGraphEstimator"]) {
    config = config["factorGraphEstimator"];
  }

  debug_ = alphapilot::get<bool>("debug", config, false);
  optimize_hz_ = alphapilot::get<double>("optimizationFrequency", config, 10.0);
  if (config["priorConfig"]) {
    YAML::Node prior_config = config["priorConfig"];
    std::vector<double> state = alphapilot::get<std::vector<double>>("initial_state", prior_config,
                                                                     {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                                                      0.0});
    prior_config_.initial_state.x = state[0];
    prior_config_.initial_state.y = state[1];
    prior_config_.initial_state.z = state[2];

    prior_config_.initial_state.qw = state[3];
    prior_config_.initial_state.qx = state[4];
    prior_config_.initial_state.qy = state[5];
    prior_config_.initial_state.qw = state[6];

    prior_config_.initial_state.x_dot = state[7];
    prior_config_.initial_state.y_dot = state[8];
    prior_config_.initial_state.z_dot = state[9];

    prior_config_.initial_vel_noise = alphapilot::get<double>("initial_vel_noise", prior_config, 0.1);
    prior_config_.initial_bias_noise = alphapilot::get<double>("initial_bias_noise", prior_config, 0.1);
    prior_config_.initial_pose_noise = alphapilot::get<std::vector<double>>("initial_pose_noise", prior_config,
                                                                            {0.05, 0.05, 0.05, 0.25, 0.25, 0.25});
  }

  // Construct parameters for ISAM optimizer
  if (config["isamParameters"]) {
    YAML::Node isam_config = config["isamParameters"];
    isam_parameters_.relinearizeThreshold = alphapilot::get<double>("relinearizeThreshold", isam_config, 0.01);
    isam_parameters_.relinearizeSkip = alphapilot::get<int>("relinearizeSkip", isam_config, 1);
    isam_parameters_.enablePartialRelinearizationCheck = alphapilot::get<bool>("enablePartialRelinearizationCheck",
                                                                               isam_config, false);
    isam_parameters_.cacheLinearizedFactors = alphapilot::get<bool>("cacheLinearizedFactors", isam_config, false);
    isam_parameters_.enableDetailedResults = alphapilot::get<bool>("enableDetailedResults", isam_config, true);
    isam_parameters_.findUnusedFactorSlots = alphapilot::get<bool>("findUnusedFactorSlots", isam_config, false);
    //ISAM2DoglegParams optimizationParams;
    gtsam::ISAM2GaussNewtonParams optimizationParams;
    optimizationParams.wildfireThreshold = alphapilot::get<double>("gaussianWildfireThreshold", isam_config, 0.001);
    isam_parameters_.optimizationParams = optimizationParams;
    isam_parameters_.print();
  }
  isam_ = ISAM2(isam_parameters_);

  if (config["cameraFactorParams"]) {
    YAML::Node camera_config = config["cameraFactorParams"];
    // camera
    use_camera_factors_ = alphapilot::get<bool>("useCameraFactor", camera_config, false);
    double default_noise = alphapilot::get<double>("defaultPixelNoise", camera_config, 10.0);
    default_camera_noise_ = noiseModel::Isotropic::Sigma(2, default_noise);  // one pixel in u and v

    if(!alphapilot::get<bool>("useDegenerateSolutions", camera_config, false)) {
      projection_params_.setDegeneracyMode(DegeneracyMode::ZERO_ON_DEGENERACY);
    }
    projection_params_.setLandmarkDistanceThreshold(alphapilot::get<double>("landmarkDistanceThreshold",
            camera_config, 10.0));
    projection_params_.setDynamicOutlierRejectionThreshold(alphapilot::get<double>(
            "dynamicOutlierRejectionThreshold", camera_config, 10.0));

    // load cameras
    std::vector<std::string> camera_names = alphapilot::get<std::vector<std::string>>("cameraNames", camera_config, {});
    for (auto &camera_name : camera_names) {
      std::shared_ptr<alphapilot::transform> trans = std::make_shared<alphapilot::transform>();
      std::vector<double> transform = alphapilot::get<std::vector<double>>("transform", camera_config[camera_name],
                                                                           {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0});
      trans->x = transform[0];
      trans->y = transform[1];
      trans->z = transform[2];

      trans->qw = transform[3];
      trans->qx = transform[4];
      trans->qy = transform[5];
      trans->qz = transform[6];

      std::shared_ptr<alphapilot::camera_info> cam_info = std::make_shared<alphapilot::camera_info>();

      std::vector<double> K = alphapilot::get<std::vector<double>>("K", camera_config[camera_name],
                                                                   {0.0, 0.0, 0.0, 1.0, 0.0});

      cam_info->fx = K[0];
      cam_info->fy = K[1];
      cam_info->s = K[2];
      cam_info->u0 = K[3];
      cam_info->v0 = K[4];

      register_camera(camera_name, trans, cam_info);
    }

    std::vector<std::string> object_names = alphapilot::get<std::vector<std::string>>("objects", camera_config, {});
    for (auto &object_name : object_names) {
      double noise = alphapilot::get<double>(object_name + "_noise", camera_config, default_noise);
      gtsam::noiseModel::Diagonal::shared_ptr
          noise_model = noiseModel::Isotropic::Sigma(2, noise);  // one pixel in u and v
      object_noises_.insert(std::pair<std::string, gtsam::noiseModel::Diagonal::shared_ptr>(object_name, noise_model));
    }
  }

  if (config["imuFactorParams"]) {
    YAML::Node imu_config = config["imuFactorParams"];

    // IMU
    use_imu_factors_ = alphapilot::get<bool>("useImuFactor", imu_config, false);
    use_imu_bias_ = alphapilot::get<bool>("useImuBias", imu_config, false);
    invert_x_ = alphapilot::get<bool>("invertX", imu_config, false);
    invert_y_ = alphapilot::get<bool>("invertY", imu_config, false);
    invert_z_ = alphapilot::get<bool>("invertZ", imu_config, false);

    imu_bias_incr_ = alphapilot::get<int>("imuBiasIncr", imu_config, 2);
    Vector6 covvec;
    std::vector<double> covvec_arr = alphapilot::get<std::vector<double>>("biasNoise", imu_config,
                                                                          {0.1, 0.1, 0.1, 0.1, 0.1, 0.1});
    covvec << covvec_arr[0], covvec_arr[1], covvec_arr[2], covvec_arr[3], covvec_arr[4], covvec_arr[5];
    bias_noise_ = noiseModel::Diagonal::Variances(covvec);

    // setup IMU preintegrator parameters
    boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params>
        p = PreintegratedCombinedMeasurements::Params::MakeSharedU(GRAVITY);
    double accel_noise_sigma = alphapilot::get<double>("accelNoiseSigma", imu_config, 2.0);
    double gyro_noise_sigma = alphapilot::get<double>("gyroNoiseSigma", imu_config, 0.1);
    double accel_bias_rw_sigma = alphapilot::get<double>("accelBiasRwSigma", imu_config, 0.1);
    double gyro_bias_rw_sigma = alphapilot::get<double>("gyroBiasRwSigma", imu_config, 0.1);
    Matrix33 measured_acc_cov = Matrix33::Identity(3, 3) * pow(accel_noise_sigma, 2);
    Matrix33 measured_omega_cov = Matrix33::Identity(3, 3) * pow(gyro_noise_sigma, 2);
    // error committed in integrating position from velocities
    Matrix33 integration_error_cov =
        Matrix33::Identity(3, 3) * alphapilot::get<double>("integrationErrorCov", imu_config, 1e-4);
    Matrix33 bias_acc_cov = Matrix33::Identity(3, 3) * pow(accel_bias_rw_sigma, 2);
    Matrix33 bias_omega_cov = Matrix33::Identity(3, 3) * pow(gyro_bias_rw_sigma, 2);
    // error in the bias used for preintegration
    Matrix66 bias_acc_omega_int = Matrix::Identity(6, 6) * alphapilot::get<double>("biasAccOmegaInt", imu_config, 0.1);

    // add them to the params
    p->accelerometerCovariance = measured_acc_cov; // acc white noise in continuous
    p->integrationCovariance = integration_error_cov; // integration uncertainty continuous
    p->gyroscopeCovariance = measured_omega_cov; // gyro white noise in continuous
    p->biasAccCovariance = bias_acc_cov; // acc bias in continuous
    p->biasOmegaCovariance = bias_omega_cov; // gyro bias in continuous
    p->biasAccOmegaInt = bias_acc_omega_int;

    // use regular preintegrated
    preintegrator_imu_ = PreintegratedImuMeasurements(p, gtsam::imuBias::ConstantBias());
  }

  if (config["poseFactorParams"]) {
    YAML::Node pose_config = config["poseFactorParams"];
    // pose factor noise
    use_pose_factors_ = alphapilot::get<bool>("usePoseFactor", pose_config, true);
    std::vector<double>
        odom_vel_arr = alphapilot::get<std::vector<double>>("poseVelNoise", pose_config, {0.3, 0.3, 0.3});
    Vector3 odom_vel_noise = Vector3(odom_vel_arr[0], odom_vel_arr[1], odom_vel_arr[2]);
    std::cout << "odom vel noise = " << odom_vel_noise << std::endl;
    odometry_vel_noise_ = noiseModel::Diagonal::Sigmas(odom_vel_noise);
    Vector6 pose_noise;
    std::vector<double> pose_noise_arr = alphapilot::get<std::vector<double>>("poseNoise", pose_config,
                                                                              {0.25, 0.25, 0.25, 0.25, 0.25, 0.25});
    pose_noise
        << pose_noise_arr[0], pose_noise_arr[1], pose_noise_arr[2], pose_noise_arr[3], pose_noise_arr[4], pose_noise_arr[5];
    std::cout << "pose noise = " << pose_noise << std::endl;
    odometry_pose_noise_ = noiseModel::Diagonal::Sigmas(pose_noise);
  }

  // add the priors on state
  add_priors(prior_config_.initial_state);
}

/**
 * callback for the imu, integrates the reading using the preintegrator_imu_, and then
 * calls propagate_imu to update the estimated state given to GTSAM
 * @param imu_data, assumes the accel and angular vel was over the last dt seconds
 */
void FactorGraphEstimator::callback_imu(const std::shared_ptr<IMU_readings> imu_data) {
  if(debug_) {
    //std::cout << __func__ << std::endl;
  }
  if (!use_imu_factors_) {
    return;
  }
  double dt = 0;
  if (last_imu_time_ == -1) {
    last_imu_time_ = imu_data->time;
    return;
  }
  dt = imu_data->time - last_imu_time_;
  last_imu_time_ = imu_data->time;
  if (dt <= 0) {
    std::cout << "ERROR: cannot use imu reading with dt <= 0" << std::endl;
    return;
  }
  position_update_ = true;
  // -1 if invert, 1 if !invert
  double invert_x = invert_x_ ? -1.0 : 1.0;
  double invert_y = invert_y_ ? -1.0 : 1.0;
  double invert_z = invert_z_ ? -1.0 : 1.0;

  // updates the current guesses of position
  Vector3 accel = Vector3(imu_data->x_accel * invert_x, imu_data->y_accel * invert_y, imu_data->z_accel * invert_z);
  // TODO units??
  if(use_imu_bias_) {
    accel += current_bias_guess_.accelerometer();
  }
  Vector3
      ang_rate = Vector3(imu_data->roll_vel * invert_x, imu_data->pitch_vel * invert_y, imu_data->yaw_vel * invert_z);
  if(use_imu_bias_) {
    ang_rate += current_bias_guess_.gyroscope();
  }
  lock_guard<mutex> preintegration_lck(preintegrator_lck_);
  preintegrator_imu_.integrateMeasurement(accel, ang_rate, dt);
  imu_meas_count_++;

  // integrate the IMU to get an updated estimate of the current position
  // integrate the imu reading using affine dynamics from TODO cite
  propagate_imu(accel, ang_rate, dt);
}

/**
 * Creates a new state variable by increasing the state index_
 * and adds it to the graph.
 * The basic process for doing so is creating a factor relating
 * previous state variables to current state variables (aka. motion model)
 * and then adding a guess for the current values of those states
 * to give the optimizer a value to start from
 */
void FactorGraphEstimator::add_imu_factor() {
  if(debug_) {
    std::cout << __func__ << std::endl;
  }
  if (imu_meas_count_ <= 0) {
    return;
  }

  // Update bias at a slower rate
  lock_guard<mutex> lck(graph_lck_);
  if (index_ % imu_bias_incr_ == 0) {
    bias_index_++;
    Symbol b1 = symbol_shorthand::B(bias_index_ - 1);
    Symbol b2 = symbol_shorthand::B(bias_index_);

    // Motion model of bias - currently constant bias is assumed

    // Add factor to graph and add initial variable guesses
    current_incremental_graph_->emplace_shared<BetweenFactor<
        imuBias::ConstantBias>>(b1, b2, imuBias::ConstantBias(),
                                bias_noise_);
    gtsam_current_state_initial_guess_->insert(symbol_shorthand::B(bias_index_),
                                              current_bias_guess_);
  }

  preintegrator_lck_.lock();
  // motion model of position and velocity variables
  std::cout << "creating IMU index at = " << index_ << std::endl;
  ImuFactor imufac(symbol_shorthand::X(index_ - 1),
                   symbol_shorthand::V(index_ - 1),
                   symbol_shorthand::X(index_),
                   symbol_shorthand::V(index_),
                   symbol_shorthand::B(bias_index_),
                   preintegrator_imu_);

  // clear IMU rolling integration
  preintegrator_imu_.resetIntegration();
  imu_meas_count_ = 0;
  preintegrator_lck_.unlock();

  // Add factor to graph and add initial variable guesses
  std::cout << "IMU guess at index = " << index_ << std::endl;
  gtsam_current_state_initial_guess_->insert(symbol_shorthand::X(index_),
                                            current_position_guess_);
  gtsam_current_state_initial_guess_->insert(symbol_shorthand::V(index_),
                                            current_velocity_guess_);
  // prevents current_vel and pos from being updated
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
                                       landmark_data, std::string camera_name) {

  // TODO verify that the landmark data is in the camera FOV
  if(debug_) {
    std::cout << __func__ << std::endl;
  }

  // if there is no position updates this is unconstrained
  if (!position_update_) {
    std::cerr << "ERROR: no position updates have happened but camera was recieved\n" <<
              "Not optimizing otherwise unconstrained optimization" << std::endl;
    return;
  }
  if (camera_map.find(camera_name) == camera_map.end()) {
    std::cout << "ERROR using invalid camera name " << camera_name << " make sure to register a camera first"
              << std::endl;
  }
  // Create newest state
  add_factors();

  if (!use_camera_factors_) {
    return;
  }
  for (const auto &seen_landmark : *landmark_data) {
    std::string l_id = seen_landmark.first;
    pair<double, double> im_coords = seen_landmark.second;
    std::string object_type = l_id.substr(0, l_id.find('-'));

    auto landmark_factor_it = landmark_factors_.find(l_id);

    // New Landmark - Add a new Factor
    if (landmark_factor_it == landmark_factors_.end()) {
      gtsam_camera camera = camera_map[camera_name];

      SmartProjectionPoseFactor<Cal3_S2>::shared_ptr smartFactor;
      if (object_noises_.find(l_id) != object_noises_.end()) {
        smartFactor = boost::make_shared<SmartProjectionPoseFactor<Cal3_S2>>(
            object_noises_[object_type], camera.K, camera.transform, projection_params_);
      } else {
        smartFactor = boost::make_shared<SmartProjectionPoseFactor<Cal3_S2>>(
            default_camera_noise_, camera.K, camera.transform, projection_params_);
      }

      landmark_factors_[l_id] = smartFactor;

      graph_lck_.lock();
      current_incremental_graph_->push_back(smartFactor);
      graph_lck_.unlock();
    }
    // Translate detection into gtsam
    Point2 detection_coords(im_coords.first, im_coords.second);

    // Add landmark to factor
    graph_lck_.lock();
    detection detection;
    detection.index = index_;
    detection.point = detection_coords;
    detection.id = l_id;
    detections_.push_back(detection);
    graph_lck_.unlock();
  }
}

std::map<std::string, std::array<double, 3>> FactorGraphEstimator::get_landmark_positions() {
  // TODO return variable, set up elsewhere when optimize is called
  if(debug_) {
    //std::cout << __func__ << std::endl;
  }
  lock_guard<mutex> graph_lck(graph_lck_);
  std::map<std::string, std::array<double, 3>> result;
  for(auto it = landmark_factors_.begin(); it != landmark_factors_.end(); it++) {
    boost::optional<Point3> point = it->second->point();
    if(point) {
      std::array<double, 3> xyz = {point.get()[0], point.get()[1], point.get()[2]};
      result.insert(std::pair<std::string, std::array<double, 3>>(it->first, xyz));
    }
  }
  return result;
}

/**
 * Adds the local factor graph that was constructed during the most recent
 * callbacks, then clears the graph for the next iteration.
 * This optimizes the entire state trajectory
 */
void FactorGraphEstimator::run_optimize() {
  if(debug_) {
    std::cout << __func__ << std::endl;
  }
  if(!optimization_lck_.try_lock()) {
    return;
  }
  if (current_incremental_graph_->empty()) {
    //std::cerr << "ERROR: cannot optimize over a empty graph" << std::endl;
    optimization_lck_.unlock();
    return;
  }
  position_update_ = false;

  // run the optimization and output the final state
  graph_lck_.lock();
  latest_state_lck_.lock();
  std::cout << "starting optimization at " << index_ << std::endl;
  int temp_index = index_;
  int temp_bias_index = bias_index_;
  drone_state previous_state = current_pose_estimate_;
  Pose3 previous_guess_pose = current_position_guess_;
  latest_state_lck_.unlock();

  std::shared_ptr<gtsam::NonlinearFactorGraph> graph_copy = current_incremental_graph_;
  std::shared_ptr<gtsam::Values> guesses_copy = gtsam_current_state_initial_guess_;

  // clear incremental graph and current guesses
  current_incremental_graph_ = std::make_shared<gtsam::NonlinearFactorGraph>();
  gtsam_current_state_initial_guess_ = std::make_shared<gtsam::Values>();

  // add the camera factors
  std::cout << detections_.size() << " held back detections" << std::endl;
  for(const detection& temp : detections_) {
    landmark_factors_[temp.id]->add(temp.point, symbol_shorthand::X(temp.index));
  }
  detections_.clear();

  graph_lck_.unlock();

  if (debug_) {
    history_.insert(*guesses_copy);
    std::cout << "\nguess state guesses" << std::endl;
    guesses_copy->print();
    std::cout << "\nincremental graph with error" << std::endl;
    graph_copy->printErrors(history_);
    std::cout << "\nlandmark factors have " << landmark_factors_.size() << " active factors" << std::endl;
    for (auto it = landmark_factors_.begin(); it != landmark_factors_.end(); it++) {
      std::cout << "factor = " << it->first << std::endl;
      it->second->print();
      std::cout << "error " << it->second->error(history_) << "\n\n" << std::endl;
    }
  }

  auto start = std::chrono::steady_clock::now();

  // run update and run optimization
  isam_.update(*graph_copy, *guesses_copy);
  //for(int i = 0; i < 1000000000; i++) {
  //  asm("");
  //}

  auto end = std::chrono::steady_clock::now();
  std::chrono::duration<double> diff = end - start;
  optimization_time_ = diff.count();

  std::cout << "getting locks for updating optimization" << std::endl;
  if (!use_imu_factors_) {
    pose_lck_.lock();
  }
  preintegrator_lck_.lock();
  graph_lck_.lock();
  latest_state_lck_.lock();
  std::cout << "done getting locks for updating optimization" << std::endl;

  // get the current state
  drone_state current_state = current_pose_estimate_;
  std::cout << "current Z = " << current_state.z << std::endl;
  std::cout << "previous Z = " << previous_state.z << std::endl;

  // calculate the how much the state has changed since we started the optimization
  gtsam::Quaternion odom_q = gtsam::Quaternion(current_state.qw, current_state.qx,
          current_state.qy, current_state.qz);
  odom_q.normalize();
  gtsam::Quaternion last_pose_q(previous_state.qw, previous_state.qx,
          previous_state.qy, previous_state.qz);
  last_pose_q.normalize();

  Rot3 rot_update = Rot3() * traits<gtsam::Quaternion>::Between(last_pose_q, odom_q);
  Point3 trans_update = Point3(current_state.x - previous_state.x, current_state.y - previous_state.y,
                               current_state.z - previous_state.z);
  Pose3 update = Pose3(rot_update, trans_update);
  std::cout << "update pos = " << update << std::endl;
  std::cout << "update vel = " <<  gtsam::Vector3(current_state.x_dot, current_state.y_dot, current_state.z_dot)
          - gtsam::Vector3(previous_state.x_dot, previous_state.y_dot, previous_state.z_dot) << std::endl;

  // set the guesses of state to the correct output, update to account for changes since last optimization
  std::cout << "temp index = " << temp_index << std::endl;
  Pose3 local_position_guess = isam_.calculateEstimate<Pose3>(symbol_shorthand::X(temp_index));
  current_position_guess_.print("\nposition before\n");

  trans_update = Point3(previous_state.x + trans_update.x(),
          previous_state.y + trans_update.y(), previous_state.z + trans_update.z());
  // TODO clean up
  //current_position_guess_ = Pose3(Rot3(rot_update * odom_q), trans_update);
  current_position_guess_ = previous_guess_pose * update;
  current_position_guess_ = Pose3(current_position_guess_.rotation(), trans_update);
  current_position_guess_.print("\nposition after\n");

  current_velocity_guess_ = isam_.calculateEstimate<Vector3>(symbol_shorthand::V(temp_index));
  std::cout << "\nvelocity before = " << current_velocity_guess_.transpose() << std::endl;
  current_velocity_guess_ += gtsam::Vector3(current_state.x_dot, current_state.y_dot, current_state.z_dot)
          - gtsam::Vector3(previous_state.x_dot, previous_state.y_dot, previous_state.z_dot);
  std::cout << "\nvelocity after = " << current_velocity_guess_.transpose() << std::endl;
  if (use_imu_factors_) {
    current_bias_guess_ = isam_.calculateEstimate<imuBias::ConstantBias>(symbol_shorthand::B(temp_bias_index));
  }

  //if (debug_) {
    std::cout << "\ncurrent position guess" << std::endl;
    current_position_guess_.print();
    std::cout << "\ncurrent velocity guess" << std::endl;
    std::cout << current_velocity_guess_ << std::endl;
    if (use_imu_factors_) {
      current_bias_guess_.print("\nCurrent bias guess");
    }
  //}

  graph_lck_.unlock();
  preintegrator_lck_.unlock();
  if (!use_imu_factors_) {
    pose_lck_.unlock();
  }

  // set the result to best guess
  current_pose_estimate_.x = current_position_guess_.x();
  current_pose_estimate_.y = current_position_guess_.y();
  current_pose_estimate_.z = current_position_guess_.z();

  current_pose_estimate_.qw = current_position_guess_.rotation().quaternion()[0];
  current_pose_estimate_.qx = current_position_guess_.rotation().quaternion()[1];
  current_pose_estimate_.qy = current_position_guess_.rotation().quaternion()[2];
  current_pose_estimate_.qz = current_position_guess_.rotation().quaternion()[3];

  current_pose_estimate_.x_dot = current_velocity_guess_[0];
  current_pose_estimate_.y_dot = current_velocity_guess_[1];
  current_pose_estimate_.z_dot = current_velocity_guess_[2];
  latest_state_lck_.unlock();
  optimization_lck_.unlock();
}

/**
 * Propagates the state according to the (stochastic) rigid body dynamics equations
 * Modifies class members
 * @param acc linear acceleration in body frame
 * @param angular_vel in body frame
 * @param dt time difference
 */
 // under preintegration lock
void FactorGraphEstimator::propagate_imu(Vector3 acc, Vector3 angular_vel, double dt) {
  if(debug_) {
    //std::cout << __func__ << std::endl;
  }
  //TODO fix IMU motion model to match what is happening in real life
  drone_state result;

  double x = current_position_guess_.x();
  double y = current_position_guess_.y();
  double z = current_position_guess_.z();

  double x_dot = current_velocity_guess_[0];
  double y_dot = current_velocity_guess_[1];
  double z_dot = current_velocity_guess_[2];

  double roll = current_position_guess_.rotation().rpy()[0];
  double pitch = current_position_guess_.rotation().rpy()[1];
  double yaw = current_position_guess_.rotation().rpy()[2];

  double roll_dot = angular_vel[0];
  double pitch_dot = angular_vel[1];
  double yaw_dot = angular_vel[2];

  /*
  if (debug_) {
    std::cout.precision(10);
    std::cout << "===== prop =====" << std::endl;
    std::cout << "dt = " << dt << std::endl;
    std::cout << "ang rate before = " << roll_dot << ", " << pitch_dot << ", " << yaw_dot << std::endl;
    std::cout << "acc beore = " << acc[0] << ", " << acc[1] << ", " << acc[2] << std::endl;
    std::cout << "=====" << std::endl;

    std::cout << "pos before = " << x << ", " << y << ", " << z << std::endl;
    std::cout << "rpy before = " << roll << ", " << pitch << ", " << yaw << std::endl;
    std::cout << "vel before = " << x_dot << ", " << y_dot << ", " << z_dot << std::endl;
  }
  */

  //Update angular rate
  result.roll_dot = roll_dot;
  result.pitch_dot = pitch_dot;
  result.yaw_dot = yaw_dot;

  //Update velocity
  float c_phi, c_theta, c_psi, s_phi, s_theta, s_psi;
  float ux, uy, uz;
  // TODO convert to quaternions
  c_phi = cosf(roll);
  c_theta = cosf(pitch);
  c_psi = cosf(yaw);
  s_phi = sinf(roll);
  s_theta = sinf(pitch);
  s_psi = sinf(yaw);
  ux = acc[0] * dt;
  uy = acc[1] * dt;
  uz = acc[2] * dt;
  result.x_dot = x_dot + (c_theta * c_psi) * ux + (s_phi * s_theta * c_psi - c_phi * s_psi) * uy
      + (c_phi * s_theta * c_psi + s_phi * s_psi) * uz;
  result.y_dot = y_dot + (c_theta * s_psi) * ux + (s_phi * s_theta * s_psi + c_phi * c_psi) * uy
      + (c_phi * s_theta * s_psi - s_phi * c_psi) * uz;
  result.z_dot = z_dot + (-s_theta) * ux + (c_theta * s_phi) * uy + (c_theta * c_phi) * uz - GRAVITY * dt;

  //position update
  result.x = x + (result.x_dot + x_dot) / 2 * dt;
  result.y = y + (result.y_dot + y_dot) / 2 * dt;
  result.z = z + (result.z_dot + z_dot) / 2 * dt;

  //Update the euler angles
  float r_result, p_result, y_result;
  r_result = roll + (roll_dot + (s_phi * s_theta / c_theta) * pitch_dot + (c_phi * s_theta / c_theta) * yaw_dot) * dt;
  p_result = pitch + (c_phi * pitch_dot - s_phi * yaw_dot) * dt;
  y_result = yaw + (s_phi / c_theta * pitch_dot + c_phi / c_theta * yaw_dot) * dt;

  // apply the update
  // the ordering is correct for gtsam
  current_position_guess_ = Pose3(Rot3::Ypr(y_result, p_result, r_result),
                                  Point3(result.x, result.y, result.z));
  current_velocity_guess_ = Vector3(result.x_dot, result.y_dot, result.z_dot);

  // prop state forward
  // set the result to initial guess
  latest_state_lck_.lock();
  current_pose_estimate_.x = current_position_guess_.x();
  current_pose_estimate_.y = current_position_guess_.y();
  current_pose_estimate_.z = current_position_guess_.z();

  current_pose_estimate_.qw = current_position_guess_.rotation().quaternion()[0];
  current_pose_estimate_.qx = current_position_guess_.rotation().quaternion()[1];
  current_pose_estimate_.qy = current_position_guess_.rotation().quaternion()[2];
  current_pose_estimate_.qz = current_position_guess_.rotation().quaternion()[3];

  current_pose_estimate_.x_dot = current_velocity_guess_[0];
  current_pose_estimate_.y_dot = current_velocity_guess_[1];
  current_pose_estimate_.z_dot = current_velocity_guess_[2];
  latest_state_lck_.unlock();

  /*
  if (debug_) {
    std::cout << "ux = " << ux << ", uy = " << uy << ", uz = " << uz << std::endl;
    std::cout << "===== after ====" << std::endl;
    std::cout << "pos after = " << result.x << ", " << result.y << ", " << result.z << std::endl;
    std::cout << "rpy after = " << r_result << ", " << p_result << ", " << y_result << std::endl;
    std::cout << "vel after = " << result.x_dot << ", " << result.y_dot << ", " << result.z_dot << std::endl;
  }
  */
}

void FactorGraphEstimator::callback_range(int rangestuff) {
  if(debug_) {
    std::cout << __func__ << std::endl;
  }
  if (!use_range_factors_) {
    return;
  }

}

// under no locks
void FactorGraphEstimator::add_priors(const drone_state &initial_state) {
  if(debug_) {
    std::cout << __func__ << std::endl;
  }
  gtsam_current_state_initial_guess_ = std::make_shared<gtsam::Values>();
  current_incremental_graph_ = std::make_shared<gtsam::NonlinearFactorGraph>();

  current_position_guess_ =
      Pose3(Rot3::Quaternion(initial_state.qw, initial_state.qx, initial_state.qy, initial_state.qz),
            Point3(initial_state.x, initial_state.y, initial_state.z));
  current_velocity_guess_ = Vector3(initial_state.x_dot, initial_state.y_dot, initial_state.z_dot);

  // init the set of Values passed to GTSAM during optimization
  vel_change_accum_ = Vector3(0, 0, 0);

  Vector6 bias_tmp;
  bias_tmp << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  std::cout << "bias temp = " << current_bias_guess_ << std::endl;
  current_bias_guess_ = imuBias::ConstantBias(bias_tmp);

  // insert initial guesses of state
  gtsam_current_state_initial_guess_->insert(symbol_shorthand::X(index_),
                                            current_position_guess_);
  gtsam_current_state_initial_guess_->insert(symbol_shorthand::V(index_),
                                            current_velocity_guess_);
  if (use_imu_factors_) {
    gtsam_current_state_initial_guess_->insert(symbol_shorthand::B(bias_index_),
                                              current_bias_guess_);
  }
  // create priors on the state

  // Assemble prior noise model and add it the graph.
  std::vector<double> pose_noise_arr = prior_config_.initial_pose_noise;
  noiseModel::Diagonal::shared_ptr pose_noise_model = noiseModel::Diagonal::Sigmas((Vector(6)
      << pose_noise_arr[0], pose_noise_arr[1], pose_noise_arr[2], pose_noise_arr[3], pose_noise_arr[4], pose_noise_arr[5]).finished()); // rad,rad,rad,m, m, m
  if (debug_) {
    std::cout << "\npose noise prior pointer = " << std::endl;
    std::cout << pose_noise_model->sigmas();
  }

  noiseModel::Diagonal::shared_ptr
      velocity_noise_model = noiseModel::Isotropic::Sigma(3, prior_config_.initial_vel_noise);
  if (debug_) {
    std::cout << "\nvel prior pointer = " << std::endl;
    std::cout << velocity_noise_model->sigmas();
  }
  noiseModel::Diagonal::shared_ptr bias_noise_model = noiseModel::Isotropic::Sigma(6, prior_config_.initial_bias_noise);
  if (debug_) {
    std::cout << "\nbias_noise_model prior = " << std::endl;
    std::cout << bias_noise_model->sigmas();
  }
  // priors match initial guess
  current_incremental_graph_->add(PriorFactor<Pose3>(symbol_shorthand::X(index_),
                                                    current_position_guess_,
                                                    pose_noise_model));
  current_incremental_graph_->add(PriorFactor<Vector3>(symbol_shorthand::V(index_),
                                                      current_velocity_guess_,
                                                      velocity_noise_model));
  if (use_imu_factors_) {
    current_incremental_graph_->add(PriorFactor<imuBias::ConstantBias>(symbol_shorthand::B(bias_index_),
                                                                      current_bias_guess_,
                                                                      bias_noise_model));
  }

  // set the result to initial guess
  current_pose_estimate_.x = current_position_guess_.x();
  current_pose_estimate_.y = current_position_guess_.y();
  current_pose_estimate_.z = current_position_guess_.z();

  current_pose_estimate_.qw = current_position_guess_.rotation().quaternion()[0];
  current_pose_estimate_.qx = current_position_guess_.rotation().quaternion()[1];
  current_pose_estimate_.qy = current_position_guess_.rotation().quaternion()[2];
  current_pose_estimate_.qz = current_position_guess_.rotation().quaternion()[3];

  current_pose_estimate_.x_dot = current_velocity_guess_[0];
  current_pose_estimate_.y_dot = current_velocity_guess_[1];
  current_pose_estimate_.z_dot = current_velocity_guess_[2];

  last_pose_state_ = current_pose_estimate_;

  run_optimize();
}

void FactorGraphEstimator::resetGraph(const drone_state &initial_state) {
  {
    if(debug_) {
      std::cout << __func__ << std::endl;
    }
    lock_guard<mutex> pose_lck(pose_lck_);
    lock_guard<mutex> preintegration_lck(preintegrator_lck_);
    lock_guard<mutex> graph_lck(graph_lck_);
    lock_guard<mutex> latest_state_lck(latest_state_lck_);

    //clear
    index_ = 0;
    bias_index_ = 0;
    pose_message_count_ = 0;
    imu_meas_count_ = 0;
    current_incremental_graph_ = std::make_shared<gtsam::NonlinearFactorGraph>();
    gtsam_current_state_initial_guess_->clear();
    preintegrator_imu_.resetIntegration();
    isam_ = ISAM2(isam_parameters_);
    history_.clear();
  }

  // reset up priors
  add_priors(initial_state);
}

drone_state FactorGraphEstimator::latest_state(bool optimize) {
  if(debug_) {
    //std::cout << __func__ << std::endl;
  }
  if (position_update_ && optimize) {
    run_optimize();
  }
  lock_guard<mutex> graph_lck(latest_state_lck_);
  // set the result to best guess
  current_pose_estimate_.x = current_position_guess_.x();
  current_pose_estimate_.y = current_position_guess_.y();
  current_pose_estimate_.z = current_position_guess_.z();

  current_pose_estimate_.qw = current_position_guess_.rotation().quaternion()[0];
  current_pose_estimate_.qx = current_position_guess_.rotation().quaternion()[1];
  current_pose_estimate_.qy = current_position_guess_.rotation().quaternion()[2];
  current_pose_estimate_.qz = current_position_guess_.rotation().quaternion()[3];

  current_pose_estimate_.x_dot = current_velocity_guess_[0];
  current_pose_estimate_.y_dot = current_velocity_guess_[1];
  current_pose_estimate_.z_dot = current_velocity_guess_[2];
  return current_pose_estimate_;
}

/**
 * Takes in a global estimate of pose, finds the difference and accumulates the delta for the factor
 * @param odom_data
 */
void FactorGraphEstimator::callback_odometry(const std::shared_ptr<drone_state> odom_data) {
  if(debug_) {
    //std::cout << __func__ << std::endl;
  }
  if (!use_pose_factors_) {
    return;
  }
  // otherwise you can have a between factor of two states that do not exist
  if (!use_imu_factors_) {
    position_update_ = true;
  }
  lock_guard<mutex> pose_lck(pose_lck_);

  pose_message_count_++;

  gtsam::Quaternion odom_q = gtsam::Quaternion(odom_data->qw, odom_data->qx, odom_data->qy, odom_data->qz);
  odom_q.normalize();
  gtsam::Quaternion last_pose_q(last_pose_state_.qw, last_pose_state_.qx, last_pose_state_.qy, last_pose_state_.qz);
  last_pose_q.normalize();

  // find the difference between the two quaternions
  Rot3 rot_update = traits<gtsam::Quaternion>::Between(last_pose_q, odom_q);
  pose_rot_accum_ = pose_rot_accum_ * rot_update;

  Point3 trans_update = Point3(odom_data->x - last_pose_state_.x, odom_data->y - last_pose_state_.y,
                               odom_data->z - last_pose_state_.z);
  pose_trans_accum_ += trans_update;

  Vector3 vel_update(odom_data->x_dot - last_pose_state_.x_dot, odom_data->y_dot - last_pose_state_.y_dot,
                     odom_data->z_dot - last_pose_state_.z_dot);

  vel_change_accum_ += vel_update;

  last_pose_state_ = *odom_data;

}

void FactorGraphEstimator::add_pose_factor() {
  if(debug_) {
    std::cout << __func__ << std::endl;
  }
  if (pose_message_count_ <= 0) {
    return;
  }
  lock_guard<mutex> pose_lck(pose_lck_);
  lock_guard<mutex> graph_lck(graph_lck_);
  pose_message_count_ = 0;

  // set up the state guesses from odometry if imu is turned off
  if (!use_imu_factors_) {
    // set up guesses before transforming to body frame for between factor
    Rot3 new_rot = pose_rot_accum_ * current_position_guess_.rotation();

    Point3 new_point = current_position_guess_.translation() + pose_trans_accum_;
    gtsam_current_state_initial_guess_->insert(symbol_shorthand::X(index_),
                                              Pose3(new_rot, new_point));

    Vector3 temp_vel = current_velocity_guess_ + vel_change_accum_;
    gtsam_current_state_initial_guess_->insert(symbol_shorthand::V(index_), temp_vel);
  }



  // add constraint on the poses
  // transforms the position offset into body frame offset
  Point3 body_trans = current_position_guess_.transformTo(pose_trans_accum_ + current_position_guess_.translation());


  // assumes that the pose change is in body frame
  current_incremental_graph_->emplace_shared<BetweenFactor<Pose3>>(symbol_shorthand::X(index_ - 1),
                                                                  symbol_shorthand::X(index_),
                                                                  Pose3(pose_rot_accum_, body_trans),
                                                                  odometry_pose_noise_);

  // add constraint on the velocity
  current_incremental_graph_->emplace_shared<BetweenFactor<Vector3>>(symbol_shorthand::V(index_ - 1),
                                                                    symbol_shorthand::V(index_),
                                                                    vel_change_accum_,
                                                                    odometry_vel_noise_);
  /*
  if(debug_) {
    std::cout << "====== POSE FACTOR =======" << std::endl;
    std::cout << "vel = " << vel_change_accum_.transpose() << std::endl;
    std::cout << "rot = " << pose_rot_accum_ << std::endl;
    std::cout << "translations = " << pose_trans_accum_ << std::endl;
  }
  */

  vel_change_accum_ = Vector3(0, 0, 0);
  pose_rot_accum_ = Rot3();
  pose_trans_accum_ = Point3(0, 0, 0);
}

void FactorGraphEstimator::add_factors() {
  if(debug_) {
    std::cout << __func__ << std::endl;
  }
  if (!position_update_) {
    return;
  }
  index_++;
  add_pose_factor();
  add_imu_factor();
}

void FactorGraphEstimator::register_camera(const std::string name,
                                           const std::shared_ptr<transform> transform,
                                           const std::shared_ptr<camera_info> camera_info) {
  if(debug_) {
    //std::cout << __func__ << std::endl;
  }
  if (name.empty()) {
    std::cout << "ERROR: invalid name of camera cannot be empty" << std::endl;
  }
  // TODO validate the camera intrinsics make sense
  gtsam_camera cam;
  // x focal, y focal, skew, center in x, center in y
  cam.K = boost::make_shared<gtsam::Cal3_S2>(camera_info->fx,
                                             camera_info->fy,
                                             camera_info->s,
                                             camera_info->u0,
                                             camera_info->v0);
  cam.transform = Pose3(Rot3::Quaternion(transform->qw, transform->qx, transform->qy, transform->qz),
                        Point3(transform->x, transform->y, transform->z));
  camera_map.insert(std::pair<std::string, gtsam_camera>(name, cam));
}

double FactorGraphEstimator::get_optimization_time() {
  return optimization_time_;
}

double FactorGraphEstimator::optimize_hz() {
  return optimize_hz_;
}

} // estimator
} // StateEstimator
