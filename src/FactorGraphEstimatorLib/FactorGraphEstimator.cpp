#include <StateEstimator/FactorGraph/FactorGraphEstimator.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>

#include <yaml-cpp/yaml.h>
#include <unordered_set>
#include <queue>

using namespace std;
using namespace gtsam;

namespace estimator {
  using symbol_shorthand::X; // robot Pose3 (x,y,z,r,p,y)
  using symbol_shorthand::V; // robot linear velocity global frame? (vx,vy,vz)
  using symbol_shorthand::B; // robot IMU/Gyro bias
  using symbol_shorthand::G; // transform to the GPS
  using symbol_shorthand::C; // transform to the cameras
  using symbol_shorthand::L; // Point3 of a landmark that is being tracked with prior
  using symbol_shorthand::A; // variables that correspond to an aruco marker, prior optional

FactorGraphEstimator::FactorGraphEstimator(const std::string &config_file) {

  if(debug_) {
    std::cout << "loading config file from " << config_file << std::endl;
  }

  YAML::Node config = YAML::LoadFile(config_file);

  current_state_guess_ = std::make_shared<gtsam::Values>();
  history_ = std::make_shared<gtsam::Values>();
  current_incremental_graph_ = std::make_shared<gtsam::NonlinearFactorGraph>();

  if (config["factorGraphEstimator"]) {
    config = config["factorGraphEstimator"];
  }

  debug_ = estimation_utils::get<bool>("debug", config, false);
  if(debug_) {
    std::cout << "Running in debug mode" << std::endl;
  }
  if (config["priorConfig"]) {
    YAML::Node prior_config = config["priorConfig"];
    std::vector<double> state = estimation_utils::get<std::vector<double>>("initial_state", prior_config,
                                                                     {0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                                                      0.0});
    gtsam::Point3 point(state[0], state[1], state[2]);
    gtsam::Rot3 orientation(state[3], state[4], state[5], state[6]);

    prior_config_.state = gtsam::Pose3(orientation, point);

    prior_config_.velocity = gtsam::Vector3(state[7], state[8], state[9]);

    prior_config_.initial_vel_noise = estimation_utils::get<double>("initial_vel_noise", prior_config, 0.1);
    prior_config_.initial_pose_noise = estimation_utils::get<std::vector<double>>("initial_pose_noise", prior_config,
                                                                            {0.05, 0.05, 0.05, 0.25, 0.25, 0.25});
  }

  // Construct parameters for ISAM optimizer
  if (config["isamParameters"]) {
    YAML::Node isam_config = config["isamParameters"];
    isam_parameters_.relinearizeThreshold = estimation_utils::get<double>("relinearizeThreshold", isam_config, 0.01);
    isam_parameters_.relinearizeSkip = estimation_utils::get<int>("relinearizeSkip", isam_config, 1);
    isam_parameters_.enablePartialRelinearizationCheck = estimation_utils::get<bool>("enablePartialRelinearizationCheck",
                                                                               isam_config, false);
    isam_parameters_.enableRelinearization = estimation_utils::get<bool>("enableRelinearization", isam_config, true);
    isam_parameters_.cacheLinearizedFactors = estimation_utils::get<bool>("cacheLinearizedFactors", isam_config, false);
    isam_parameters_.findUnusedFactorSlots = estimation_utils::get<bool>("findUnusedFactorSlots", isam_config, false);
    if(debug_) {
      isam_parameters_.enableDetailedResults = true;
      isam_parameters_.evaluateNonlinearError = true;
    }
    std::string optimization_method = estimation_utils::get<std::string>("optimizer", isam_config, std::string("GN"));
    if(optimization_method == "DL") {
      YAML::Node dl_config = isam_config["isamParametersDL"];
      gtsam::ISAM2DoglegParams optimizationParams;
      optimizationParams.verbose = debug_;
      optimizationParams.wildfireThreshold = estimation_utils::get<double>("wildfireThreshold", dl_config, 1e-5);
      optimizationParams.initialDelta = estimation_utils::get<double>("initialDelta", dl_config, 1.0);
      std::string adaptationMode = estimation_utils::get<std::string>("trustRegionAdaptionMode", dl_config, std::string("oneStep"));
      if(adaptationMode == "searchEachIter") {
        optimizationParams.adaptationMode = gtsam::DoglegOptimizerImpl::SEARCH_EACH_ITERATION;
      } else if(adaptationMode == "searchReduce") {
        optimizationParams.adaptationMode = gtsam::DoglegOptimizerImpl::SEARCH_REDUCE_ONLY;
      } else {
        if(adaptationMode != "oneStep") {
          std::cout << "WARNING: adaptionMode in invalid " << adaptationMode << " is not in set {searchEachIter, searchReduce, oneStep}\n" <<
                                                                       " defaulting to oneStep" << std::endl;
        }
        optimizationParams.adaptationMode = gtsam::DoglegOptimizerImpl::ONE_STEP_PER_ITERATION;
      }
      isam_parameters_.optimizationParams = optimizationParams;

    } else {
      if (optimization_method != "GN") {
        std::cout << "WARNING: optimization method " << optimization_method << " not valid of set {GN, DL}\n"
                                                                              "using GaussNewton" << std::endl;
      }
      gtsam::ISAM2GaussNewtonParams optimizationParams;
      optimizationParams.wildfireThreshold = estimation_utils::get<double>("gaussianWildfireThreshold", isam_config, 0.001);
      isam_parameters_.optimizationParams = optimizationParams;
    }
    isam_parameters_.print();
  } else {
    std::cerr << "cannot find isamParameters, using all defaults" << std::endl;
    exit(1);
  }
  isam_ = ISAM2(isam_parameters_);

  if (config["cameraConfig"]) {
    YAML::Node camera_config = config["cameraConfig"];

    pairing_threshold_ = estimation_utils::get<double>("pairingThreshold", camera_config, 0.1);

    // load cameras
    std::vector<std::string> camera_names = estimation_utils::get<std::vector<std::string>>("cameraNames", camera_config, {});
    for (auto &camera_name : camera_names) {
      std::vector<double> translation_vec = estimation_utils::get<std::vector<double>>("translation", camera_config[camera_name],
                                                                           {0.0, 0.0, 0.0});

      std::shared_ptr<gtsam::Point3> translation = std::make_shared<gtsam::Point3>(translation_vec[0], translation_vec[1], translation_vec[2]);

      std::vector<double> rotation_vec = estimation_utils::get<std::vector<double>>("rotation", camera_config[camera_name],
                                                                              {1,0,0,0,1,0,0,0,1});
      std::shared_ptr<Rot3> rotation;
      if(rotation_vec.size() == 9) {
        rotation = std::make_shared<Rot3>(rotation_vec[0], rotation_vec[1], rotation_vec[2],
                                          rotation_vec[3], rotation_vec[4], rotation_vec[5], rotation_vec[6], rotation_vec[7], rotation_vec[8]);
      } else if(rotation_vec.size() == 4) {
        rotation = std::make_shared<Rot3>(rotation_vec[0], rotation_vec[1], rotation_vec[2], rotation_vec[3]);
      }
      std::shared_ptr<sensor_msgs::CameraInfo> cam_info = std::make_shared<sensor_msgs::CameraInfo>();

      std::vector<double> K = estimation_utils::get<std::vector<double>>("K", camera_config[camera_name],
                                                                   {0.0, 0.0, 0.0, 1.0, 0.0});

      cam_info->K[0] = K[0];
      cam_info->K[4] = K[1];
      // TODO check
      //cam_info-> = K[2] / camera_scale_factor;
      cam_info->K[2] = K[3];
      cam_info->K[5] = K[4];

      RegisterCamera(camera_name, translation, rotation, cam_info);

    }
  } else {
    std::cout << "cannot find any camera configuration exiting" << std::endl;
    exit(-1);
  }

  /*
  if (config["projectionFactorParams"]) {
    YAML::Node camera_config = config["projectionFactorParams"];
    // camera
    use_projection_factors_ = estimation_utils::get<bool>("useProjectionFactor", camera_config, false);
    double default_noise = estimation_utils::get<double>("defaultPixelNoise", camera_config, 10.0);
    default_camera_noise_ = noiseModel::Isotropic::Sigma(2, default_noise);  // one pixel in u and v

    projection_landmark_noise_ = noiseModel::Isotropic::Sigma(3, projection_constraint_noise_val_);

    std::vector<std::string> object_names = estimation_utils::get<std::vector<std::string>>("objects", camera_config, {});
    for (auto &object_name : object_names) {
      double noise = estimation_utils::get<double>(object_name + "_noise", camera_config["objectList"], default_noise);
      gtsam::noiseModel::Diagonal::shared_ptr
          noise_model = noiseModel::Isotropic::Sigma(2, noise);  // one pixel in u and v
      object_noises_.insert(std::pair<std::string, gtsam::noiseModel::Diagonal::shared_ptr>(object_name, noise_model));
    }

    Vector3 prior_noise;
    std::vector<double> prior_noise_arr = estimation_utils::get<std::vector<double>>("objectPriorNoise", camera_config, {0.1, 0.1, 0.1});
    prior_noise << prior_noise_arr[0], prior_noise_arr[1], prior_noise_arr[2];
    landmark_prior_noise_ = noiseModel::Diagonal::Sigmas(prior_noise);
    // load the priors of the factors I care about
    std::vector<std::string> tracked_objects = estimation_utils::get<std::vector<std::string>>(
            "trackedObjects", camera_config, {});
    std::cout << "adding " << tracked_objects.size() << " tracked objects to the factor graph" << std::endl;
    for(auto &object_name : tracked_objects) {
      if(!camera_config["objectPriors"][object_name]) {
        std::cout << "WARNING: cannot find prior for object " << object_name << std::endl;
      }
      std::shared_ptr<Landmark> new_landmark = std::make_shared<Landmark>();
      std::vector<double> location = estimation_utils::get<std::vector<double>>(object_name, camera_config["objectPriors"], {});
      new_landmark->point.x = location[0];
      new_landmark->point.y = location[1];
      new_landmark->point.z = location[2];
      new_landmark->id = object_name.substr(0, object_name.find("_"));
      new_landmark->type = object_name.substr(object_name.find("_")+1, std::string::npos);
      AddProjectionPrior(new_landmark);

    }

  }
   */

  if (config["arucoFactorParams"]) {
    YAML::Node aruco_config = config["arucoFactorParams"];

    use_aruco_factors_ = estimation_utils::get<bool>("useArucoFactor", aruco_config, true);
    use_aruco_constraints_ = estimation_utils::get<bool>("useArucoConstraints", aruco_config, true);
    use_aruco_prior_ = estimation_utils::get<bool>("useArucoPrior", aruco_config, true);
    use_range_for_aruco_ = estimation_utils::get<bool>("useArucoRange", aruco_config, true);
    use_projection_debug_ = estimation_utils::get<bool>("useProjectionDebug", aruco_config, false);
    aruco_length_ = estimation_utils::get<double>("arucoLength", aruco_config, 0.2);

    double aruco_range_noise = estimation_utils::get<double>("arucoRangeNoise", aruco_config, 1.0);
    aruco_range_noise_ = gtsam::noiseModel::Isotropic::Sigma(1, aruco_range_noise);
    aruco_constraint_noise_ = gtsam::noiseModel::Constrained::All(3);

    Vector6 aruco_noise;
    std::vector<double> aruco_noise_arr = estimation_utils::get<std::vector<double>>("arucoNoise", aruco_config,
                                                                              {0.25, 0.25, 0.25, 0.25, 0.25, 0.25});
    aruco_noise
        << aruco_noise_arr[0], aruco_noise_arr[1], aruco_noise_arr[2], aruco_noise_arr[3], aruco_noise_arr[4], aruco_noise_arr[5];
    double aruco_camera_noise = estimation_utils::get<double>("projectionPixelNoise", aruco_config, 10.0);
    aruco_camera_noise_ = noiseModel::Isotropic::Sigma(2, aruco_camera_noise);

    std::vector<double> aruco_pose_prior_noise_arr = estimation_utils::get<std::vector<double>>("arucoPriorNoise", aruco_config,
                                                                              {0.25, 0.25, 0.25});
    Vector3 aruco_pose_prior_noise;
    aruco_pose_prior_noise << aruco_pose_prior_noise_arr[0], aruco_pose_prior_noise_arr[1], aruco_pose_prior_noise_arr[2];
    aruco_pose_prior_noise_ = noiseModel::Diagonal::Sigmas(aruco_pose_prior_noise);

    // load priors for all of the aruco markers
    std::vector<std::string> aruco_prior_ids = estimation_utils::get<std::vector<std::string>>(
            "arucoIds", aruco_config, {});
    for(std::string aruco_id : aruco_prior_ids) {
      if(!aruco_config["arucoIdsLocations"][aruco_id]) {
        std::cout << "WARNING: cannot find prior for aruco id " << aruco_id << std::endl;
        continue;
      }
      std::vector<double> location = estimation_utils::get<std::vector<double>>(aruco_id, aruco_config["arucoIdsLocations"], {});
      AddArucoPrior(location, stoi(aruco_id));
    }
  }

  if (config["imuFactorParams"]) {
    YAML::Node imu_config = config["imuFactorParams"];

    // IMU
    imu_debug_ = estimation_utils::get<bool>("imuDebug", imu_config, false);
    use_imu_factors_ = estimation_utils::get<bool>("useImuFactor", imu_config, false);
    use_imu_bias_ = estimation_utils::get<bool>("useImuBias", imu_config, false);

    invert_x_ = estimation_utils::get<bool>("invertX", imu_config, false);
    invert_y_ = estimation_utils::get<bool>("invertY", imu_config, false);
    invert_z_ = estimation_utils::get<bool>("invertZ", imu_config, false);

    imu_bias_incr_ = estimation_utils::get<int>("imuBiasIncr", imu_config, 2);
    Vector6 covvec;
    std::vector<double> covvec_arr = estimation_utils::get<std::vector<double>>("biasNoise", imu_config,
                                                                          {0.1, 0.1, 0.1, 0.1, 0.1, 0.1});
    covvec << covvec_arr[0], covvec_arr[1], covvec_arr[2], covvec_arr[3], covvec_arr[4], covvec_arr[5];
    bias_noise_ = noiseModel::Diagonal::Variances(covvec);

    // setup IMU preintegrator parameters
    boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params>
            p = PreintegratedCombinedMeasurements::Params::MakeSharedU(GRAVITY);
    double accel_noise_sigma = estimation_utils::get<double>("accelNoiseSigma", imu_config, 2.0);
    double gyro_noise_sigma = estimation_utils::get<double>("gyroNoiseSigma", imu_config, 0.1);
    double accel_bias_rw_sigma = estimation_utils::get<double>("accelBiasRwSigma", imu_config, 0.1);
    double gyro_bias_rw_sigma = estimation_utils::get<double>("gyroBiasRwSigma", imu_config, 0.1);
    Matrix33 measured_acc_cov = Matrix33::Identity(3, 3) * pow(accel_noise_sigma, 2);
    Matrix33 measured_omega_cov = Matrix33::Identity(3, 3) * pow(gyro_noise_sigma, 2);
    // error committed in integrating position from velocities
    Matrix33 integration_error_cov =
            Matrix33::Identity(3, 3) * estimation_utils::get<double>("integrationErrorCov", imu_config, 1e-4);
    Matrix33 bias_acc_cov = Matrix33::Identity(3, 3) * pow(accel_bias_rw_sigma, 2);
    Matrix33 bias_omega_cov = Matrix33::Identity(3, 3) * pow(gyro_bias_rw_sigma, 2);
    // error in the bias used for preintegration
    Matrix66 bias_acc_omega_int = Matrix::Identity(6, 6) * estimation_utils::get<double>("biasAccOmegaInt", imu_config, 0.1);

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
    use_pose_factors_ = estimation_utils::get<bool>("usePoseFactor", pose_config, true);
    std::vector<double>
        odom_vel_arr = estimation_utils::get<std::vector<double>>("poseVelNoise", pose_config, {0.3, 0.3, 0.3});
    Vector3 odom_vel_noise = Vector3(odom_vel_arr[0], odom_vel_arr[1], odom_vel_arr[2]);
    odometry_vel_noise_ = noiseModel::Diagonal::Sigmas(odom_vel_noise);
    Vector6 pose_noise;
    std::vector<double> pose_noise_arr = estimation_utils::get<std::vector<double>>("poseNoise", pose_config,
                                                                              {0.25, 0.25, 0.25, 0.25, 0.25, 0.25});
    pose_noise
        << pose_noise_arr[0], pose_noise_arr[1], pose_noise_arr[2], pose_noise_arr[3], pose_noise_arr[4], pose_noise_arr[5];
    odometry_pose_noise_ = noiseModel::Diagonal::Sigmas(pose_noise);
  }

  if(config["smartPoseProjectionFactors"]) {
    YAML::Node smart_pose_config = config["smartPoseProjectionFactors"];

    // use smart factors
    use_smart_pose_projection_factor_ = estimation_utils::get<bool>("useSmartPoseProjectionFactor", smart_pose_config, false);
    if(!estimation_utils::get<bool>("useDegenerateSolutions", smart_pose_config, false)) {
      projection_params_.setDegeneracyMode(DegeneracyMode::ZERO_ON_DEGENERACY);
    }
    projection_params_.setLandmarkDistanceThreshold(estimation_utils::get<double>("landmarkDistanceThreshold",
                                                                            smart_pose_config, 10.0));
    projection_params_.setDynamicOutlierRejectionThreshold(estimation_utils::get<double>(
        "dynamicOutlierRejectionThreshold", smart_pose_config, 10.0));

    projection_params_.throwCheirality = false;


    double default_noise = estimation_utils::get<double>("defaultPixelNoise", smart_pose_config, 10.0);
    smart_default_noise_ = noiseModel::Isotropic::Sigma(2, default_noise);  // one pixel in u and v
  }

  if(debug_) {
    std::cout << "\nFinished Init\n"
            << "Using IMU Factor: " << (use_imu_factors_ ? "true" : "false") << "\n"
            << "Using IMU Bias Factor: " << (use_imu_bias_ ? "true" : "false") << "\n"
            << "Using pose Factor: " << (use_pose_factors_ ? "true" : "false") << "\n"
            << "Using projection Factor: " << (use_projection_factors_ ? "true" : "false") << "\n"
            << "Using aruco Factor: " << (use_aruco_factors_ ? "true" : "false") << "\n"
            << "Using smart Factor: " << (use_smart_pose_projection_factor_ ? "true" : "false") << "\n"
            << "\nStarting at state: " << prior_config_.state << "\n\n"
            << std::endl;
  }

  // add the priors on state
  AddPriors();
}


void FactorGraphEstimator::AddPriors() {

  current_position_guess_ = prior_config_.state;
  current_velocity_guess_ = prior_config_.velocity;

  // init the set of Values passed to GTSAM during optimization
  vel_change_accum_ = Vector3(0, 0, 0);

  // insert initial guesses of state
  current_state_guess_->insert(X(index_), current_position_guess_);
  current_state_guess_->insert(V(index_), current_velocity_guess_);

  if (use_imu_factors_) {
    current_state_guess_->insert(B(bias_index_), current_bias_guess_);
  }

  // Assemble prior noise model and add it the graph.
  std::vector<double> pose_noise_arr = prior_config_.initial_pose_noise;
  noiseModel::Diagonal::shared_ptr pose_noise_model = noiseModel::Diagonal::Sigmas((Vector(6)
          << pose_noise_arr[0], pose_noise_arr[1], pose_noise_arr[2], pose_noise_arr[3], pose_noise_arr[4], pose_noise_arr[5]).finished()); // rad,rad,rad,m, m, m

  noiseModel::Diagonal::shared_ptr
          velocity_noise_model = noiseModel::Isotropic::Sigma(3, prior_config_.initial_vel_noise);

  noiseModel::Diagonal::shared_ptr bias_noise_model = noiseModel::Isotropic::Sigma(6, prior_config_.initial_bias_noise);

  // priors match initial guess
  current_incremental_graph_->add(PriorFactor<Pose3>(X(index_),
                                                     current_position_guess_,
                                                     pose_noise_model));
  current_incremental_graph_->add(PriorFactor<Vector3>(V(index_),
                                                       current_velocity_guess_,
                                                       velocity_noise_model));

  if (use_imu_factors_) {
    current_incremental_graph_->add(PriorFactor<imuBias::ConstantBias>(B(bias_index_),
                                                                       current_bias_guess_,
                                                                       bias_noise_model));
  }

  // set the result to initial guess
  current_pose_estimate_.pose.pose.position.x = current_position_guess_.x();
  current_pose_estimate_.pose.pose.position.y = current_position_guess_.y();
  current_pose_estimate_.pose.pose.position.z = current_position_guess_.z();

  current_pose_estimate_.pose.pose.orientation.w = current_position_guess_.rotation().quaternion()[0];
  current_pose_estimate_.pose.pose.orientation.x = current_position_guess_.rotation().quaternion()[1];
  current_pose_estimate_.pose.pose.orientation.y = current_position_guess_.rotation().quaternion()[2];
  current_pose_estimate_.pose.pose.orientation.z = current_position_guess_.rotation().quaternion()[3];

  current_pose_estimate_.twist.twist.linear.x = current_velocity_guess_[0];
  current_pose_estimate_.twist.twist.linear.y = current_velocity_guess_[1];
  current_pose_estimate_.twist.twist.linear.z = current_velocity_guess_[2];

  last_pose_state_ = current_pose_estimate_;

  RunOptimize();
}

void FactorGraphEstimator::TimingCallback(const double timestamp) {
  std::unique_lock<std::mutex> timing_lck(timing_lck_, std::try_to_lock);
  if(!timing_lck) {
    return;
  }
  if(!time_map_.empty() && timestamp <= time_map_.rbegin()->second) {
     return;
  }
  preintegrator_lck_.lock();
  if(position_update_) {
    preintegrator_lck_.unlock();
    // check if we should add a new state
    AddImuFactor();
    AddPoseFactor();
    lock_guard<mutex> graph_lck(graph_lck_);
    index_++;
    time_map_.insert(std::make_pair(index_, timestamp));
  } else {
    std::cout << "WARNING: no position update before timing callback, not creating a state" << std::endl;
  }
}

std::vector<Landmark> FactorGraphEstimator::GetLandmarkPositions() {
  lock_guard<mutex> landmark_lck(landmark_lck_);
  std::vector<Landmark> result;
  for(auto it = landmark_locations_.begin(); it != landmark_locations_.end(); it++) {
    result.push_back(it->second);
  }
  return result;
}

void FactorGraphEstimator::PrintValues(std::shared_ptr<gtsam::Values> values, std::string prefix) {
  std::cout << prefix;
  KeyFormatter keyFormatter = gtsam::DefaultKeyFormatter;
  std::cout << "\nValues with " << values->size() << " values:" << std::endl;
  for(auto key = values->begin(); key != values->end(); key++) {
    cout << "Value (" << keyFormatter(key->key) << "): ";
    Symbol symbol = Symbol(key->key);
    if(symbol.chr() == 'x') {
      Pose3 temp = key->value.cast<Pose3>();
      std::cout << temp << "\n";
    } else if(symbol.chr() == 'v') {
      Vector3 temp = key->value.cast<Vector3>();
      std::cout << temp.transpose() << "\n";
    } else if(symbol.chr() == 'l' || symbol.chr() == 'a' || symbol.chr() == 'g') {
      Point3 temp = key->value.cast<Point3>();
      std::cout << temp.transpose() << "\n";
    } else if(symbol.chr() == 'b') {
      gtsam::imuBias::ConstantBias temp = key->value.cast<imuBias::ConstantBias>();
      std::cout << temp << "\n";
    }
  }
  std::cout << std::endl;
}

/**
 * Adds the local factor graph that was constructed during the most recent
 * callbacks, then clears the graph for the next iteration.
 * This optimizes the entire state trajectory
 */
bool FactorGraphEstimator::RunOptimize() {
  std::unique_lock<std::mutex> opt_lck_guard(optimization_lck_, std::try_to_lock);
  if(!opt_lck_guard.owns_lock()) {
    std::cout << "WARNING: trying to have multiple optimziations of factor graph running at the same time" << std::endl;
    return false;
  }
  if (current_incremental_graph_->empty()) {
    optimization_lck_.unlock();
    return false;
  }

  // run the optimization and output the final state
  std::unique_lock<std::mutex> graph_lck_guard(graph_lck_);
  // stores the current state of the local graph so that we can continue to build a local graph while optimizing
  int temp_index = index_;
  int temp_bias_index = bias_index_;
  /*
  if(debug_) {
    std::cout << "\nstarting optimization with indexes up to = " << temp_index << std::endl;
  }
   */

  std::shared_ptr<gtsam::NonlinearFactorGraph> graph_copy = current_incremental_graph_;
  std::shared_ptr<gtsam::Values> guesses_copy = current_state_guess_;

  // clear incremental graph and current guesses
  current_incremental_graph_ = std::make_shared<gtsam::NonlinearFactorGraph>();
  current_state_guess_ = std::make_shared<gtsam::Values>();
  // clear anything in imu queue
  imu_queue_.clear();

  graph_lck_guard.unlock();

  smart_detections_lck_.lock();

  while(!smart_detections_queue_.empty()) {
    SmartDetection detection = smart_detections_queue_.front();
    /*
    if(debug_) {
      std::cout << "adding detection on camera: " << detection.header.camera << " with id: "
              << detection.header.id << " and type: " << detection.header.type
              << "\nat state index " << detection.state_index << " of " << detection.detection << std::endl;
    }
     */
    // TODO should also add camera key detection came from
    id_to_smart_landmarks_[detection.header]->add(detection.detection, X(detection.state_index));
    smart_detections_queue_.pop_front();
  }
  smart_detections_lck_.unlock();

  history_->insert(*guesses_copy);

  /*
  if (debug_) {
    std::cout << "======= BEFORE error ======\n";
    PrintValues(guesses_copy, "guess state guesses");
    std::cout << "\nincremental graph with error" << std::endl;
    graph_copy->printErrors(history_);
    for(auto it = id_to_smart_landmarks_.begin(); it != id_to_smart_landmarks_.end(); it++) {
      if(it->second->error(history_) > 0) {
        it->second->print();
        std::cout << "error: " << it->second->error(history_) << std::endl;
      }
    }
  }
  */

  auto start = std::chrono::steady_clock::now();

  // run update and run optimization
  ISAM2Result results = isam_.update(*graph_copy, *guesses_copy);
  auto temp_start = std::chrono::steady_clock::now();
  Values optimized_values = isam_.calculateEstimate();
  auto temp_end = std::chrono::steady_clock::now();
  std::chrono::duration<double> temp_diff = temp_end - temp_start;
  if(debug_) {
    std::cout << "time to calculate estimate: " << temp_diff.count() << std::endl;
  }
  // print out optimizations statistics
  optimization_stats_.variables_reeliminated = results.variablesReeliminated;
  optimization_stats_.variables_relinearized = results.variablesRelinearized;
  if(results.errorBefore) {
    optimization_stats_.error_before = *results.errorBefore;
  } else {
    optimization_stats_.error_before = -1;
  }
  if(results.errorAfter) {
    optimization_stats_.error_after = *results.errorAfter;
  } else {
    optimization_stats_.error_after = -1;
  }

  history_->update(optimized_values);

  /*
  if(debug_) {
    std::cout << "======= AFTER error ======\n";
    //optimized_values.print();
    graph_copy->printErrors(history_);
  }
   */

  // get timing statistics
  ++optimization_stats_.optimization_iterations;
  double optimization_iterations = optimization_stats_.optimization_iterations;
  auto end = std::chrono::steady_clock::now();
  std::chrono::duration<double> diff = end - start;
  double optimization_time = diff.count();
  optimization_stats_.optimization_time_avg = (optimization_iterations - 1.0)/optimization_iterations*optimization_stats_.optimization_time_avg +
                                            optimization_time/optimization_iterations;
  optimization_stats_.optimization_time = optimization_time;

  start = std::chrono::steady_clock::now();


  std::unique_lock<std::mutex> landmark_lck_guard(landmark_lck_);
  // calculate the locations of the landmarks
  landmark_locations_.clear();
  for(auto it = landmark_to_id_map_.begin(); it != landmark_to_id_map_.end(); it++) {
    Point3 landmark = optimized_values.at<Point3>(L(it->second));
    //print_projection(temp_index, landmark, camera_map_["left_right"], Point2(0,0));
    Landmark new_landmark;
    new_landmark.point.x = landmark.x();
    new_landmark.point.y = landmark.y();
    new_landmark.point.z = landmark.z();
    new_landmark.id = it->first.id;
    new_landmark.type = it->first.type;
    landmark_locations_.insert(std::make_pair(it->second, new_landmark));

    // TODO publish covariance
  }
  landmark_lck_guard.unlock();

  // calculate the centers of aruco
  aruco_locations_lck_.lock();
  aruco_locations_.clear();
  aruco_locations_.reserve(aruco_indexes_.size() * 4);
  for(int id_base : aruco_indexes_) {
    int num_points = 0;
    for(int i = 0; i < 4; i++) {
      if (optimized_values.exists(A(id_base + i))) {
        Point3 aruco = optimized_values.at<Point3>(A(id_base + i));
        geometry_msgs::PoseWithCovarianceStamped pos;
        pos.pose.pose.position.x = aruco.x();
        pos.pose.pose.position.y = aruco.y();
        pos.pose.pose.position.z = aruco.z();

        Matrix aruco_cov = isam_.marginalCovariance(A(id_base + i));
        for(int j = 0; j < 3; j++) {
          for(int k = 0; k < 3; k++) {
            pos.pose.covariance[j * 3 + k] = aruco_cov(j, k);
          }
        }

        aruco_locations_.push_back(pos);
        num_points++;
      } else {
        std::cout << "Aruco " << id_base << i << " does not exist in isam" << std::endl;
      }
    }
  }
  aruco_locations_lck_.unlock();

  smart_locations_lck_.lock();
  smart_detections_lck_.lock();

  // get the locations tracked by the smartPoseProjectionFactor
  smart_locations_.clear();
  for(auto it = id_to_smart_landmarks_.begin(); it != id_to_smart_landmarks_.end(); it++) {
    boost::optional<Point3> point = it->second->point();
    // if the point exists and is valid
    if(point) {
      geometry_msgs::PoseWithCovarianceStamped msg;
      msg.pose.pose.position.x = point->x();
      msg.pose.pose.position.y = point->y();
      msg.pose.pose.position.z = point->z();
      if(smart_locations_.find(it->first.type) == smart_locations_.end()) {
         // insert into map
         smart_locations_.insert(std::make_pair(it->first.type, std::vector<geometry_msgs::PoseWithCovarianceStamped>()));
      }
      smart_locations_[it->first.type].push_back(msg);
    }
  }
  smart_detections_lck_.unlock();
  smart_locations_lck_.unlock();

  end = std::chrono::steady_clock::now();
  diff = end - start;
  optimization_time = diff.count();
  optimization_stats_.get_landmarks_time_avg = (optimization_iterations - 1.0)/optimization_iterations*optimization_stats_.get_landmarks_time_avg +
                                            optimization_time/optimization_iterations;
  optimization_stats_.get_landmarks_time = optimization_time;

  /*
  if(debug_) {
    std::cout << optimization_stats_ << std::endl;
  }*/

  // set the guesses of state to the correct output, update to account for changes since last optimization

  // optimized means the most recently optimized node (most recent states that have just been optimized but not applied)
  //std::cout << "optimized values: " << std::endl;
  //optimized_values.print();
  Pose3 optimized_pose = optimized_values.at<Pose3>(X(temp_index));
  Matrix pose_cov = isam_.marginalCovariance(X(temp_index));
  Vector3 optimized_vel = optimized_values.at<Vector3>(V(temp_index));
  Matrix vel_cov = isam_.marginalCovariance(V(temp_index));
  if (use_imu_bias_ && use_imu_factors_) {
    current_bias_guess_ = optimized_values.at<imuBias::ConstantBias>(B(temp_bias_index));
  }
  /*
  if(debug_) {
    std::cout << "\ntemp_index = " << temp_index << "\n"
              << "optimized Position\n" << optimized_pose << "\n"
              << "optimized Velocity: " << optimized_vel.transpose() << std::endl;
  }
   */

  graph_lck_guard.lock();
  std::lock_guard<std::mutex> preintegration_lck(preintegrator_lck_);
  std::lock_guard<std::mutex> latest_state_lck_guard(latest_state_lck_);

  // fix the accumulated guesses

  KeyFormatter keyFormatter = gtsam::DefaultKeyFormatter;
  gtsam::NavState input(optimized_pose.rotation(), optimized_pose.translation(), optimized_vel);
  for(auto it = imu_queue_.begin(); it != imu_queue_.end(); it++) {
    gtsam::Key fixed_pose_key = it->key3();
    gtsam::Key fixed_vel_key = it->key4();
    if(debug_) {
      std::cout << keyFormatter(fixed_pose_key) << std::endl;
    }
    gtsam::NavState result = it->preintegratedMeasurements().predict(input, current_bias_guess_);
    current_state_guess_->update(fixed_pose_key, result.pose());
    current_state_guess_->update(fixed_vel_key, result.v());
    input = result;
  }

  // fix the latest state
  gtsam::NavState result = preintegrator_imu_.predict(input, current_bias_guess_);
  current_position_guess_ = result.pose();
  current_velocity_guess_ = result.v();

  /*
  if(debug_) {
    std::cout << "\ncurrent position after fix\n" << current_position_guess_ << "\n";
    std::cout << "\ncurrent velocity after fix = " << current_velocity_guess_.transpose() << "\n\n\n" << std::endl;
  }
   */

  graph_lck_guard.unlock();

  // set the result to best guess
  current_pose_estimate_.pose.pose.position.x = current_position_guess_.x();
  current_pose_estimate_.pose.pose.position.y = current_position_guess_.y();
  current_pose_estimate_.pose.pose.position.z = current_position_guess_.z();

  current_pose_estimate_.pose.pose.orientation.w = current_position_guess_.rotation().quaternion()[0];
  current_pose_estimate_.pose.pose.orientation.x = current_position_guess_.rotation().quaternion()[1];
  current_pose_estimate_.pose.pose.orientation.y = current_position_guess_.rotation().quaternion()[2];
  current_pose_estimate_.pose.pose.orientation.z = current_position_guess_.rotation().quaternion()[3];

  current_pose_estimate_.twist.twist.linear.x = current_velocity_guess_[0];
  current_pose_estimate_.twist.twist.linear.x = current_velocity_guess_[1];
  current_pose_estimate_.twist.twist.linear.z = current_velocity_guess_[2];

  // set covariance
  for(int i = 0; i < 6; i++) {
    for(int j = 0; j < 6; j++) {
      current_pose_estimate_.pose.covariance[i * 6 + j] = pose_cov(i, j);
    }
  }

  /*
  if(debug_) {
    std::cout << "===========================" << std::endl;
  }
   */

  return true;
}

/**
 * callback for the imu, integrates the reading using the preintegrator_imu_, and then
 * calls propagate_imu to update the estimated state given to GTSAM
 * @param imu_data, assumes the accel and angular vel was over the last dt seconds
 */
void FactorGraphEstimator::CallbackImu(const std::shared_ptr<sensor_msgs::Imu> imu_data) {
  if (!use_imu_factors_) {
    return;
  }

  double dt = 0;
  if (last_imu_time_ == -1) {
    last_imu_time_ = imu_data->header.stamp.toSec();
    return;
  }
  dt = imu_data->header.stamp.toSec() - last_imu_time_;
  last_imu_time_ = imu_data->header.stamp.toSec();
  if (dt <= 0 || dt > 10) {
    std::cout << "WARNING: cannot use imu reading with dt <= 0 || dt > 10" << std::endl;
    return;
  }
  // -1 if invert, 1 if !invert
  double invert_x = invert_x_ ? -1.0 : 1.0;
  double invert_y = invert_y_ ? -1.0 : 1.0;
  double invert_z = invert_z_ ? -1.0 : 1.0;

  // updates the current guesses of position
  // Readings come in NED and Factor Graph is in NWU
  Vector3 accel = Vector3(imu_data->linear_acceleration.x * invert_x, imu_data->linear_acceleration.y * invert_y,
                          imu_data->linear_acceleration.z * invert_z);
  Vector3
          ang_rate = Vector3(imu_data->angular_velocity.x * invert_x, imu_data->angular_velocity.y * invert_y,
                             imu_data->angular_velocity.z * invert_z);
  lock_guard<mutex> preintegration_lck(preintegrator_lck_);
  preintegrator_imu_.integrateMeasurement(accel, ang_rate, dt);

  if(use_imu_bias_) {
    accel -= current_bias_guess_.accelerometer();
  }
  if(use_imu_bias_) {
    ang_rate -= current_bias_guess_.gyroscope();
  }

  // integrate the IMU to get an updated estimate of the current position
  if(use_imu_prop_) {
    Pose3 result_state;
    Vector3 result_vel;
    latest_state_lck_.lock();

    PropagateImu(current_position_guess_, current_velocity_guess_, preintegrator_imu_);
    position_update_ = true;

    // set the result to initial guess
    current_pose_estimate_.header.stamp = ros::Time(last_imu_time_);
    current_pose_estimate_.pose.pose.position.x = current_position_guess_.x();
    current_pose_estimate_.pose.pose.position.y = current_position_guess_.y();
    current_pose_estimate_.pose.pose.position.z = current_position_guess_.z();

    current_pose_estimate_.pose.pose.orientation.w = current_position_guess_.rotation().quaternion()[0];
    current_pose_estimate_.pose.pose.orientation.x = current_position_guess_.rotation().quaternion()[1];
    current_pose_estimate_.pose.pose.orientation.y = current_position_guess_.rotation().quaternion()[2];
    current_pose_estimate_.pose.pose.orientation.z = current_position_guess_.rotation().quaternion()[3];

    current_pose_estimate_.twist.twist.linear.x = current_velocity_guess_[0];
    current_pose_estimate_.twist.twist.linear.y = current_velocity_guess_[1];
    current_pose_estimate_.twist.twist.linear.z = current_velocity_guess_[2];

    current_pose_estimate_.twist.twist.angular.x = ang_rate.x();
    current_pose_estimate_.twist.twist.angular.y = ang_rate.y();
    current_pose_estimate_.twist.twist.angular.z = ang_rate.z();
    latest_state_lck_.unlock();
  }
}

/**
 * Propagates the state according to the (stochastic) rigid body dynamics equations
 * Modifies class members
 * @param acc linear acceleration in body frame
 * @param angular_vel in body frame
 * @param dt time difference
 */
// under preintegration lock
void FactorGraphEstimator::PropagateImu(gtsam::Pose3& pose, gtsam::Vector3& vel,
                                        gtsam::PreintegratedImuMeasurements& preintegrator) {
  gtsam::NavState input(pose.rotation(), pose.translation(),vel);
  gtsam::NavState result = preintegrator.predict(input, current_bias_guess_);
  pose = result.pose();
  vel = result.v();
}

/**
 * Creates a new state variable by increasing the state index_
 * and adds it to the graph.
 * The basic process for doing so is creating a factor relating
 * previous state variables to current state variables (aka. motion model)
 * and then adding a guess for the current values of those states
 * to give the optimizer a value to start from
 */
void FactorGraphEstimator::AddImuFactor() {
  if (!position_update_) {
    std::cout << "Position has not been update, not adding IMU factor" << std::endl;
    return;
  }

  // Update bias at a slower rate
  lock_guard<mutex> lck(graph_lck_);
  if (use_imu_bias_ && index_ % imu_bias_incr_ == 0) {
    bias_index_++;
    Symbol b1 = B(bias_index_ - 1);
    Symbol b2 = B(bias_index_);

    // Motion model of bias - currently constant bias is assumed
    // Add factor to graph and add initial variable guesses
    current_incremental_graph_->emplace_shared<BetweenFactor<
        imuBias::ConstantBias>>(b1, b2, imuBias::ConstantBias(),
                                bias_noise_);
    current_state_guess_->insert(B(bias_index_),
                                               current_bias_guess_);
  }
  lock_guard<mutex> preintegrator_lck(preintegrator_lck_);
  // motion model of position and velocity variables
  if(debug_) {
    std::cout << "creating IMU index at = " << index_ << " to " << index_ + 1 << std::endl;
  }

  if(use_imu_prop_) {
    lock_guard<mutex> latest_state_lck(latest_state_lck_);
    // Add factor to graph and add initial variable guesses
    current_state_guess_->insert(X(index_ + 1),
                                               current_position_guess_);
    current_state_guess_->insert(V(index_ + 1),
                                               current_velocity_guess_);
  }

  ImuFactor imufac(X(index_),
                   V(index_),
                   X(index_ + 1),
                   V(index_ + 1),
                   B(bias_index_),
                   preintegrator_imu_);

  // prevents current_vel and pos from being updated
  current_incremental_graph_->add(imufac);
  imu_queue_.push_back(imufac);

  // clear IMU rolling integration
  preintegrator_imu_.resetIntegration();
  position_update_ = false;
}

// Assumes that there is only a single instance of an ID per camera
void FactorGraphEstimator::ArucoCallback(const std::shared_ptr<autorally_estimation::ArucoDetections> msg) {
  if(!use_aruco_factors_) {
    return;
  }

  TimingCallback(msg->header.stamp.toSec());

  if (camera_map_.find(msg->camera_name) == camera_map_.end()) {
    std::cout << "WARNING aruco using invalid camera name " << msg->camera_name << " make sure to register a camera first"
              << std::endl;
    return;
  }
  GtsamCamera camera = camera_map_[msg->camera_name];
  int image_index = FindCameraIndex(msg->header.stamp.toSec());
  // if there is some time delay in aruco, check to make sure we have the right image
  if(image_index == -1) {
    std::cout << "WARNING: invalid index: -1 for the camera on aruco callback time: "
              << msg->header.stamp.toSec() << " candidates: " << time_map_.size() << std::endl;
    return;
  }

  lock_guard<mutex> graph_lck(graph_lck_);


  // list of ids that have been added so there are not duplicates
  for(const autorally_estimation::ArucoDetection& detection : msg->detections) {
    int id_base = detection.id.data * 10 + 1;

    // use the first detection to init the location of the aruco marker in world frame
    if(aruco_indexes_.find(id_base) == aruco_indexes_.end()) {
      std::cout << "WARNING: Aruco with base id: " << id_base << " does not have a prior, " <<
                    "it will be ignored this time" << std::endl;
      continue;
    }
    if(use_range_for_aruco_) {
      // finds the norm
      double dist_aruco = sqrt(pow(detection.pose.position.x, 2) + pow(detection.pose.position.y, 2) + pow(detection.pose.position.z, 2));
      current_incremental_graph_->emplace_shared<RangeFactor<Pose3, Point3>>(
              X(image_index),
              A(id_base - 1),
              dist_aruco,
              aruco_range_noise_);
    }

    for(unsigned int i = 0; i < detection.detections.size(); i++) {
      GenericProjectionFactor<Pose3, Point3, Cal3_S2>::shared_ptr projectionFactor;
      Point2 detection_coords = Point2(detection.detections.at(i*2), detection.detections.at(i*2+1));
      projectionFactor = boost::make_shared<GenericProjectionFactor<Pose3, Point3, Cal3_S2>>(
              detection_coords, aruco_camera_noise_, X(image_index),
              A(id_base + i), camera.k, camera.transform);
      current_incremental_graph_->push_back(projectionFactor);

      if(use_projection_debug_) {
        Point3 point = isam_.calculateEstimate<Point3>(A(id_base + i));
        PrintProjection(image_index, point, camera, detection_coords);
      }
    }
  }

}

void FactorGraphEstimator::AddProjectionPrior(std::shared_ptr<Landmark> msg) {
  Point3 prior = Point3(msg->point.x, msg->point.y, msg->point.z);
  DetectionHeader header;
  header.type = msg->type;
  header.id = msg->id;
  landmark_to_id_map_.insert(std::make_pair(header, landmark_index_));
  if(debug_) {
    std::cout << "\nadding object prior id:" << header.id << " and type: " << header.type << " with index = "
              << landmark_index_ << "\n"
              << "Prior: " << prior.transpose() << "\n"
              << "Noise: " << landmark_prior_noise_->sigmas().transpose() << std::endl;
  }
  // add prior factor
  current_incremental_graph_->add(PriorFactor<Point3>(L(landmark_index_),
                                                prior,
                                                landmark_prior_noise_));
  int my_index = landmark_index_;
  // add initial guess as well
  current_state_guess_->insert(L(my_index), prior);
  ++landmark_index_;

  // find what other landmarks are associated with this one and relate them with between factors
  if(use_projection_constraints_) {
    for(auto it = landmark_to_id_map_.begin(); it != landmark_to_id_map_.end(); it++) {
      if(it->first.id == header.id && it->first.type != header.type) {
        std::lock_guard<std::mutex> graph_lck(graph_lck_);
        int other_index = it->second;
        Point3 other_position;
        if(current_state_guess_->exists(L(other_index))) {
          other_position = current_state_guess_->at<Point3>(L(other_index));
        } else if(history_->exists(L(other_index))) {
          other_position = history_->at<Point3>(L(other_index));
        } else {
          std::cout << "WARNING: cannot find match for landmark" << std::endl;
          continue;
        }

        Point3 diff = other_position - prior;
        if(debug_) {
          std::cout << "adding between factor " << my_index << " -> " << other_index << "\n";
          std::cout << diff.transpose() << std::endl;
        }
        current_incremental_graph_->emplace_shared<BetweenFactor<Point3>>(L(my_index),
                                                                          L(other_index),
                                                                          diff,
                                                                          projection_constraint_noise_);
      }
    }
  }
}

/**
 * Takes in a global estimate of pose, finds the difference and accumulates the delta for the factor
 * @param odom_data
 */
void FactorGraphEstimator::CallbackOdometry(const std::shared_ptr<nav_msgs::Odometry> odom_data) {
  if (!use_pose_factors_) {
    return;
  }
  // only a single factor should be updating the state
  if(!use_imu_prop_) {
    // otherwise you can have a between factor of two states that do not exist
    position_update_ = true;

    latest_state_lck_.lock();

    // set up guesses before transforming to body frame for between factor
    Rot3 new_rot = pose_rot_accum_ * current_position_guess_.rotation();

    Point3 new_point = current_position_guess_.translation() + pose_trans_accum_;

    current_position_guess_ = Pose3(new_rot, new_point);

    current_velocity_guess_ += vel_change_accum_;

    current_pose_estimate_.pose.pose.position.x = current_position_guess_.x();
    current_pose_estimate_.pose.pose.position.y = current_position_guess_.y();
    current_pose_estimate_.pose.pose.position.z = current_position_guess_.z();

    current_pose_estimate_.pose.pose.orientation.w = current_position_guess_.rotation().quaternion()[0];
    current_pose_estimate_.pose.pose.orientation.x = current_position_guess_.rotation().quaternion()[1];
    current_pose_estimate_.pose.pose.orientation.y = current_position_guess_.rotation().quaternion()[2];
    current_pose_estimate_.pose.pose.orientation.z = current_position_guess_.rotation().quaternion()[3];

    current_pose_estimate_.twist.twist.linear.x = current_velocity_guess_[0];
    current_pose_estimate_.twist.twist.linear.y = current_velocity_guess_[1];
    current_pose_estimate_.twist.twist.linear.z = current_velocity_guess_[2];
    latest_state_lck_.unlock();
  }

  lock_guard<mutex> pose_lck(pose_lck_);

  pose_message_count_++;

  gtsam::Quaternion odom_q = gtsam::Quaternion(odom_data->pose.pose.orientation.w, odom_data->pose.pose.orientation.x,
                                               odom_data->pose.pose.orientation.y, odom_data->pose.pose.orientation.z);
  odom_q.normalize();
  gtsam::Quaternion last_pose_q(last_pose_state_.pose.pose.orientation.w, last_pose_state_.pose.pose.orientation.x,
                                last_pose_state_.pose.pose.orientation.y, last_pose_state_.pose.pose.orientation.z);
  last_pose_q.normalize();

  // find the difference between the two quaternions
  Rot3 rot_update = traits<gtsam::Quaternion>::Between(last_pose_q, odom_q);
  pose_rot_accum_ = pose_rot_accum_ * rot_update;

  Point3 trans_update = Point3(odom_data->pose.pose.position.x - last_pose_state_.pose.pose.position.x,
                               odom_data->pose.pose.position.y - last_pose_state_.pose.pose.position.y,
                               odom_data->pose.pose.position.z - last_pose_state_.pose.pose.position.z);
  pose_trans_accum_ += trans_update;

  Vector3 vel_update(odom_data->twist.twist.linear.x - last_pose_state_.twist.twist.linear.x,
                     odom_data->twist.twist.linear.y - last_pose_state_.twist.twist.linear.y,
                     odom_data->twist.twist.linear.z - last_pose_state_.twist.twist.linear.z);

  vel_change_accum_ += vel_update;

  last_pose_state_ = *odom_data;

}

void FactorGraphEstimator::AddPoseFactor() {
  if (pose_message_count_ <= 0) {
    return;
  }
  lock_guard<mutex> pose_lck(pose_lck_);
  lock_guard<mutex> graph_lck(graph_lck_);
  lock_guard<mutex> latest_state_lck(latest_state_lck_);
  pose_message_count_ = 0;

  // set up guesses before transforming to body frame for between factor
  Rot3 new_rot = pose_rot_accum_ * current_position_guess_.rotation();

  Point3 new_point = current_position_guess_.translation() + pose_trans_accum_;
  current_state_guess_->insert(X(index_ + 1),
                                            Pose3(new_rot, new_point));

  Vector3 temp_vel = current_velocity_guess_ + vel_change_accum_;
  current_state_guess_->insert(V(index_ + 1), temp_vel);

  // add constraint on the poses
  // transforms the position offset into body frame offset
  Point3 body_trans = current_position_guess_.transformTo(pose_trans_accum_ + current_position_guess_.translation());


  // assumes that the pose change is in body frame
  current_incremental_graph_->emplace_shared<BetweenFactor<Pose3>>(X(index_),
                                                                  X(index_ + 1),
                                                                  Pose3(pose_rot_accum_, body_trans),
                                                                  odometry_pose_noise_);

  // add constraint on the velocity
  current_incremental_graph_->emplace_shared<BetweenFactor<Vector3>>(V(index_),
                                                                    V(index_ + 1),
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

nav_msgs::Odometry FactorGraphEstimator::LatestState(bool optimize) {
  if (position_update_ && optimize) {
    RunOptimize();
  }
  lock_guard<mutex> graph_lck(latest_state_lck_);
  // set the result to best guess
  current_pose_estimate_.pose.pose.position.x = current_position_guess_.x();
  current_pose_estimate_.pose.pose.position.y = current_position_guess_.y();
  current_pose_estimate_.pose.pose.position.z = current_position_guess_.z();

  current_pose_estimate_.pose.pose.orientation.w = current_position_guess_.rotation().quaternion()[0];
  current_pose_estimate_.pose.pose.orientation.x = current_position_guess_.rotation().quaternion()[1];
  current_pose_estimate_.pose.pose.orientation.y = current_position_guess_.rotation().quaternion()[2];
  current_pose_estimate_.pose.pose.orientation.z = current_position_guess_.rotation().quaternion()[3];

  current_pose_estimate_.twist.twist.linear.x = current_velocity_guess_[0];
  current_pose_estimate_.twist.twist.linear.y = current_velocity_guess_[1];
  current_pose_estimate_.twist.twist.linear.z = current_velocity_guess_[2];
  return current_pose_estimate_;
}

void FactorGraphEstimator::RegisterCamera(const std::string name,
                                           const std::shared_ptr<gtsam::Point3> translation,
                                           const std::shared_ptr<gtsam::Rot3> rotation,
                                           const std::shared_ptr<sensor_msgs::CameraInfo> camera_info) {
  if (name.empty()) {
    std::cout << "WARNING: invalid name of camera cannot be empty" << std::endl;
  }
  // TODO validate the camera intrinsics make sense
  GtsamCamera cam;
  // x focal, y focal, skew, center in x, center in y
  cam.k = boost::make_shared<gtsam::Cal3_S2>(camera_info->K[0],
                                             camera_info->K[4],
                                             0,
                                             camera_info->K[2],
                                             camera_info->K[5]);

  cam.transform = Pose3(*rotation, *translation);

  camera_map_.insert(std::pair<std::string, GtsamCamera>(name, cam));
}

OptimizationStats FactorGraphEstimator::GetOptimizationStats() {
  return optimization_stats_;
}

int FactorGraphEstimator::FindCameraIndex(double time) {
  if(debug_) {
    std::cout << std::setprecision(15) << "finding state close to " << time << std::endl;
  }
  // if negative dt
  if(time_map_.empty()) {
    std::cout << "WARNING: time map is empty cannot match camera frame" << std::endl;
    return -1;
  }
  if(time_map_.rbegin()->second + pairing_threshold_ < time) {
    std::cout << "WARNING: current time: " << time << " is threshold ahead of the most recent time:  "
    << time_map_.rbegin()->second + pairing_threshold_ << " no correspondence found" << std::endl;
    return -1;
  }
  // TODO check if it is in the map to begin with
  for(auto it = time_map_.rbegin(); it != time_map_.rend(); it++) {
    double dt = std::abs(it->second - time);
    if(dt < pairing_threshold_) {
      if(debug_) {
        std::cout << std::setprecision(15) << "found timestamp close " << time_map_[it->first] << std::endl;
      }
      return it->first;
    } else if(it->second + pairing_threshold_ < time) {
      return -1;
    }
  }
  return -1;
}

void FactorGraphEstimator::PrintProjection(int image_index, Point3 location, GtsamCamera camera, Point2 detection_coords) {
  // TODO throws exception
  Pose3 position;
  bool print = false;
  if(history_->exists(X(image_index))) {
    print = true;
    position = history_->at<Pose3>(X(image_index));
  }
  if(!print && current_state_guess_->exists(X(image_index))) {
    print = true;
    position = current_state_guess_->at<Pose3>(X(image_index));
  }
  if(print) {
    //std::cout << "\n\nposition " << position << std::endl;
    //std::cout << "\ncamera transform " << camera.transform << std::endl;
    try {
      position = position.compose(camera.transform);
      std::cout << "\nCamera World Position: " << position << std::endl;
      PinholeCamera<Cal3_S2> test_cam(position, *camera.k);
      Point2 measurement = test_cam.project(location);
      std::cout << "Location: " << location << "\n";
      std::cout << "Detection got: " << detection_coords << "\n"
                << "Expected     : " << measurement<< std::endl;
    } catch (gtsam::CheiralityException& cheiralityException) {
      std::cout << "WARNING: caught cheirality exception when projecting point" << std::endl;
    }

  }
}

std::vector<geometry_msgs::PoseWithCovarianceStamped> FactorGraphEstimator::GetArucoLocations() {
  lock_guard<mutex> aruco_locations_lck(aruco_locations_lck_);
  return aruco_locations_;
}

std::map<std::string, std::vector<geometry_msgs::PoseWithCovarianceStamped>> FactorGraphEstimator::GetSmartLocations() {
  std::lock_guard<std::mutex> smart_locations_lck(smart_locations_lck_);
  return smart_locations_;
}

void FactorGraphEstimator::SmartProjectionCallback(const std::shared_ptr<autorally_estimation::CameraDetections> detections) {
  // TODO this does not seperate into what camera is being used for each landmark
  if (!use_smart_pose_projection_factor_) {
    return;
  }

  TimingCallback(detections->header.stamp.toSec());

  // if there is no position updates this is unconstrained
  if (camera_map_.find(detections->header.frame_id) == camera_map_.end()) {
    std::cout << "WARNING smart using invalid camera name " << detections->header.frame_id << " make sure to register a camera first"
              << std::endl;
    return;
  }

  int image_index = FindCameraIndex(detections->header.stamp.toSec());
  if(image_index == -1) {
    std::cout << "WARNING: invalid index: -1 for the camera on detection callback" << std::endl;
    return;
  }

  // New Landmark - Add a new Factor
  GtsamCamera camera = camera_map_[detections->header.frame_id];

  for(autorally_estimation::CameraDetection detection : detections->detections) {
    DetectionHeader header;
    header.type = detection.type;
    header.id = detection.id;
    header.camera = detections->header.frame_id;

    auto landmark_factor_it = id_to_smart_landmarks_.find(header);
    if(landmark_factor_it == id_to_smart_landmarks_.end()) {
      SmartProjectionPoseFactor<Cal3_S2>::shared_ptr smartFactor;
      // TODO use the initial distance metric to set the estimate

      if (smart_object_noises_.find(detection.type) != smart_object_noises_.end()) {
        smartFactor = boost::make_shared<SmartProjectionPoseFactor<Cal3_S2>>(
            smart_object_noises_[detection.type], camera.k, camera.transform, projection_params_);
      } else {
        smartFactor = boost::make_shared<SmartProjectionPoseFactor<Cal3_S2>>(
            smart_default_noise_, camera.k, camera.transform, projection_params_);
      }

      id_to_smart_landmarks_[header] = smartFactor;

      std::lock_guard<std::mutex> graph_lck(graph_lck_);
      current_incremental_graph_->push_back(smartFactor);
    }
    std::lock_guard<std::mutex> smart_detections_lck(smart_detections_lck_);
    SmartDetection smart_det;
    smart_det.detection = gtsam::Point2(detection.x, detection.y);
    smart_det.state_index = image_index;
    smart_det.header = header;
    smart_detections_queue_.push_back(smart_det);
  }
}

// TODO pose array
geometry_msgs::PoseArray FactorGraphEstimator::GetStateHistory() {
  geometry_msgs::PoseArray result;
  result.header.stamp = ros::Time(time_map_.rbegin()->second);
  for(int i = 0; i < index_; i++) {
    if(history_->exists(X(i))) {
      Pose3 other_position = history_->at<Pose3>(X(i));
      geometry_msgs::Pose pose;
      pose.position.x = other_position.x();
      pose.position.y = other_position.y();
      pose.position.z = other_position.z();

      gtsam::Quaternion q = other_position.rotation().toQuaternion();

      pose.orientation.w = q.w();
      pose.orientation.x = q.x();
      pose.orientation.y = q.y();
      pose.orientation.z = q.z();
      result.poses.push_back(pose);
    }
  }
  return result;
}

void FactorGraphEstimator::AddArucoPrior(std::vector<double> position, int id) {
  // debug message
  if(debug_) {
    std::cout << "creating aruco with id " << id << std::endl;
  }
  int id_base = id * 10 + 1;

  // create the center of the aruco marker

  // create center prior
  gtsam::Point3 center = gtsam::Point3(position[0], position[1], position[2]);
  current_state_guess_->insert(A(id_base - 1), center);
  if(use_aruco_prior_) {
    current_incremental_graph_->add(PriorFactor<Point3>(A(id_base - 1),
                                                        center,
                                                        aruco_pose_prior_noise_));
  }


  // create each corner of the aruco with constraint factor and prior
  double yaw = position[3];
  for(unsigned int i = 0; i < 4; i++) {
    Point3 diff;
    if(i == 0) {
      double x = 0 * cos(yaw) + -aruco_length_ * sin(yaw);
      double y = -aruco_length_ * cos(yaw) + 0 * sin(yaw);
      diff = Point3(x, y, aruco_length_ / 2);

    } else if(i == 1) {
      double x = 0 * cos(yaw) + aruco_length_ * sin(yaw);
      double y = aruco_length_ * cos(yaw) + 0 * sin(yaw);
      diff = Point3(x, y, aruco_length_ / 2);

    } else if(i == 2) {
      double x = 0 * cos(yaw) + aruco_length_ * sin(yaw);
      double y = aruco_length_ * cos(yaw) + 0 * sin(yaw);
      diff = Point3(x, y, -aruco_length_ / 2);

    } else if(i == 3) {
      double x = 0 * cos(yaw) + -aruco_length_ * sin(yaw);
      double y = -aruco_length_ * cos(yaw) + 0 * sin(yaw);
      diff = Point3(x, y, -aruco_length_ / 2);

    }
    std::cout << "got rotated aruco position: " << diff << std::endl;
    Point3 prior = gtsam::Point3(center.x() + diff.x(), center.y() + diff.y(), center.z() + diff.z());
    current_state_guess_->insert(A(id_base + i), prior);

    if(use_aruco_prior_) {
      current_incremental_graph_->add(PriorFactor<Point3>(A(id_base + i),
                                                          prior,
                                                          aruco_pose_prior_noise_));
    }

    // add constraints to the graph
    if(use_aruco_constraints_) {
      current_incremental_graph_->emplace_shared<BetweenFactor<Point3>>(A(id_base - 1),
                                                                        A(id_base + i),
                                                                        diff,
                                                                        aruco_constraint_noise_);
    }

  }
  aruco_indexes_.insert(id_base);
}


} // estimator
