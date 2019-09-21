#include <StateEstimator/estimation/FactorGraph/FactorGraphEstimator.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>

#include <yaml-cpp/yaml.h>
#include <alphapilot_common/GateConstraints.h>
#include <unordered_set>
#include <queue>

using namespace std;
using namespace gtsam;

namespace alphapilot {
namespace estimator {

FactorGraphEstimator::FactorGraphEstimator(const std::string &config_file, const std::string &full_path) {

  if(debug_) {
    std::cout << "loading config file from " << config_file << std::endl;
  }

  YAML::Node config = YAML::LoadFile(config_file);

  gtsam_current_state_initial_guess_ = std::make_shared<gtsam::Values>();
  current_incremental_graph_ = std::make_shared<gtsam::NonlinearFactorGraph>();

  if (config["factorGraphEstimator"]) {
    config = config["factorGraphEstimator"];
  }

  if(alphapilot::get<bool>("pipeStdout", config, false)) {
    std::cout << "WARNING: the factor graph has taken control of stdout (cout), it will be here\n"
    << full_path+"_factor_graph_output.txt" << std::endl;
    std::freopen(std::string(full_path+"_factor_graph_output.txt").data(),"w",stdout);
  }

  debug_ = alphapilot::get<bool>("debug", config, false);
  if(debug_) {
    std::cout << "Running in debug mode" << std::endl;
  }
  if (config["priorConfig"]) {
    YAML::Node prior_config = config["priorConfig"];
    std::vector<double> state = alphapilot::get<std::vector<double>>("initial_state", prior_config,
                                                                     {0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                                                      0.0});
    prior_config_.initial_state.x = state[0];
    prior_config_.initial_state.y = state[1];
    prior_config_.initial_state.z = state[2];

    prior_config_.initial_state.qw = state[3];
    prior_config_.initial_state.qx = state[4];
    prior_config_.initial_state.qy = state[5];
    prior_config_.initial_state.qz = state[6];

    prior_config_.initial_state.x_dot = state[7];
    prior_config_.initial_state.y_dot = state[8];
    prior_config_.initial_state.z_dot = state[9];

    prior_config_.initial_vel_noise = alphapilot::get<double>("initial_vel_noise", prior_config, 0.1);
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
    gtsam::ISAM2GaussNewtonParams optimizationParams;
    optimizationParams.wildfireThreshold = alphapilot::get<double>("gaussianWildfireThreshold", isam_config, 0.001);
    isam_parameters_.optimizationParams = optimizationParams;
    isam_parameters_.print();
  }
  isam_ = ISAM2(isam_parameters_);

  if (config["cameraConfig"]) {
    YAML::Node camera_config = config["cameraConfig"];

    // load cameras
    std::vector<std::string> camera_names = alphapilot::get<std::vector<std::string>>("cameraNames", camera_config, {});
    for (auto &camera_name : camera_names) {
      std::vector<double> translation_vec = alphapilot::get<std::vector<double>>("translation", camera_config[camera_name],
                                                                           {0.0, 0.0, 0.0});

      std::shared_ptr<gtsam::Point3> translation = std::make_shared<gtsam::Point3>(translation_vec[0], translation_vec[1], translation_vec[2]);

      std::vector<double> rotation_vec = alphapilot::get<std::vector<double>>("rotation", camera_config[camera_name],
                                                                              {1,0,0,0,1,0,0,0,1});
      std::shared_ptr<Rot3> rotation = std::make_shared<Rot3>(rotation_vec[0], rotation_vec[1], rotation_vec[2],
      rotation_vec[3], rotation_vec[4], rotation_vec[5], rotation_vec[6], rotation_vec[7], rotation_vec[8]);

      std::shared_ptr<alphapilot::camera_info> cam_info = std::make_shared<alphapilot::camera_info>();

      std::vector<double> K = alphapilot::get<std::vector<double>>("K", camera_config[camera_name],
                                                                   {0.0, 0.0, 0.0, 1.0, 0.0});

      cam_info->fx = K[0];
      cam_info->fy = K[1];
      cam_info->s = K[2];
      cam_info->u0 = K[3];
      cam_info->v0 = K[4];

      register_camera(camera_name, translation, rotation, cam_info);

    }
  }

  if (config["projectionFactorParams"]) {
    YAML::Node camera_config = config["projectionFactorParams"];
    // camera
    use_camera_factors_ = alphapilot::get<bool>("useProjectionFactor", camera_config, false);
    double default_noise = alphapilot::get<double>("defaultPixelNoise", camera_config, 10.0);
    default_camera_noise_ = noiseModel::Isotropic::Sigma(2, default_noise);  // one pixel in u and v
    reassign_gate_ids_ = alphapilot::get<bool>("reassignGateIds", camera_config, true);

    pairing_threshold_ = alphapilot::get<double>("pairingThreshold", camera_config, 0.1);

    std::vector<std::string> object_names = alphapilot::get<std::vector<std::string>>("objects", camera_config, {});
    for (auto &object_name : object_names) {
      double noise = alphapilot::get<double>(object_name + "_noise", camera_config["objectList"], default_noise);
      gtsam::noiseModel::Diagonal::shared_ptr
          noise_model = noiseModel::Isotropic::Sigma(2, noise);  // one pixel in u and v
      object_noises_.insert(std::pair<std::string, gtsam::noiseModel::Diagonal::shared_ptr>(object_name, noise_model));
    }

    Vector3 prior_noise;
    std::vector<double> prior_noise_arr = alphapilot::get<std::vector<double>>("objectPriorNoise", camera_config, {0.1, 0.1, 0.1});
    prior_noise << prior_noise_arr[0], prior_noise_arr[1], prior_noise_arr[2];
    landmark_prior_noise_ = noiseModel::Diagonal::Sigmas(prior_noise);
    // load the priors of the factors I care about
    std::vector<std::string> tracked_objects = alphapilot::get<std::vector<std::string>>(
            "trackedObjects", camera_config, {});
    for(auto &object_name : tracked_objects) {
      if(!camera_config["objectPriors"][object_name]) {
        std::cout << "ERROR: cannot find prior for object " << object_name << std::endl;
      }
      std::shared_ptr<alphapilot::Landmark> new_landmark = std::make_shared<alphapilot::Landmark>();
      std::vector<double> location = alphapilot::get<std::vector<double>>(object_name, camera_config["objectPriors"], {});
      new_landmark->position.x = location[0];
      new_landmark->position.y = location[1];
      new_landmark->position.z = location[2];
      new_landmark->id = object_name.substr(0, object_name.find("_"));
      new_landmark->type = object_name.substr(object_name.find("_")+1, std::string::npos);
      add_projection_prior(new_landmark);

    }
    calculate_gate_centers();

  }

  if (config["arucoFactorParams"]) {
    YAML::Node aruco_config = config["arucoFactorParams"];

    use_aruco_factors_ = alphapilot::get<bool>("useArucoFactor", aruco_config, true);

    Vector6 aruco_noise;
    std::vector<double> aruco_noise_arr = alphapilot::get<std::vector<double>>("arucoNoise", aruco_config,
                                                                              {0.25, 0.25, 0.25, 0.25, 0.25, 0.25});
    aruco_noise
        << aruco_noise_arr[0], aruco_noise_arr[1], aruco_noise_arr[2], aruco_noise_arr[3], aruco_noise_arr[4], aruco_noise_arr[5];
    if(debug_) {
      std::cout << "aruco noise = " << aruco_noise.transpose() << std::endl;
    }
    double aruco_camera_noise = alphapilot::get<double>("projectionPixelNoise", aruco_config, 10.0);
    aruco_camera_noise_ = noiseModel::Isotropic::Sigma(2, aruco_camera_noise);

    std::vector<double> aruco_pose_prior_noise_arr = alphapilot::get<std::vector<double>>("arucoPriorNoise", aruco_config,
                                                                              {0.25, 0.25, 0.25});
    Vector3 aruco_pose_prior_noise;
    aruco_pose_prior_noise << aruco_pose_prior_noise_arr[0], aruco_pose_prior_noise_arr[1], aruco_pose_prior_noise_arr[2];
    aruco_pose_prior_noise_ = noiseModel::Diagonal::Sigmas(aruco_pose_prior_noise);
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
    if(debug_) {
      std::cout << "odom vel noise = " << odom_vel_noise.transpose() << std::endl;
    }
    odometry_vel_noise_ = noiseModel::Diagonal::Sigmas(odom_vel_noise);
    Vector6 pose_noise;
    std::vector<double> pose_noise_arr = alphapilot::get<std::vector<double>>("poseNoise", pose_config,
                                                                              {0.25, 0.25, 0.25, 0.25, 0.25, 0.25});
    pose_noise
        << pose_noise_arr[0], pose_noise_arr[1], pose_noise_arr[2], pose_noise_arr[3], pose_noise_arr[4], pose_noise_arr[5];
    if(debug_) {
      std::cout << "pose noise = " << pose_noise.transpose() << std::endl;
    }
    odometry_pose_noise_ = noiseModel::Diagonal::Sigmas(pose_noise);
  }

  #if VERBOSE
    std::cout << "VERBOSE Enabled"
  #endif

  if(debug_) {
    std::cout << "\nFinished Init\n"
            << "Using IMU Factor: " << (use_imu_factors_ ? "true" : "false") << "\n"
            << "Using pose Factor: " << (use_pose_factors_ ? "true" : "false") << "\n"
            << "Using projection Factor: " << (use_camera_factors_ ? "true" : "false") << "\n"
            << "Using aruco Factor: " << (use_aruco_factors_ ? "true" : "false") << "\n"
            << "\nStarting at state: " << prior_config_.initial_state << "\n\n"
            << std::endl;
  }

  // add the priors on state
  add_priors(prior_config_.initial_state);
}

void FactorGraphEstimator::timing_callback(const double timestamp) {
  preintegrator_lck_.lock();
  if(position_update_) {
    preintegrator_lck_.unlock();
    add_imu_factor();
    add_pose_factor();
    lock_guard<mutex> graph_lck(graph_lck_);
    index_++;
    time_map_.insert(std::make_pair(index_, timestamp));
  } else {
    preintegrator_lck_.unlock();
    std::cout << "ERROR: no position update before timing callback, not creating a state" << std::endl;
  }
}

std::vector<alphapilot::Landmark> FactorGraphEstimator::get_landmark_positions() {
  lock_guard<mutex> landmark_lck(landmark_lck_);
  std::vector<Landmark> result;
  for(auto it = landmark_locations_.begin(); it != landmark_locations_.end(); it++) {
    result.push_back(it->second);
  }
  return result;
}

void FactorGraphEstimator::assign_gate_ids(std::shared_ptr<GateDetections> detection_msg) {
  lock_guard<mutex> gate_lck(gate_lck_);

  // detected id, actual gate id, distance
  auto comp = [](std::tuple<std::string, std::string, double> a, std::tuple<std::string, std::string, double> b ) {
    return std::get<2>(a) < std::get<2>(b);
  };
  std::priority_queue<std::tuple<std::string, std::string, double>, std::vector<std::tuple<std::string, std::string, double>>, decltype(comp)> pq(comp);
  for(GateDetection detection : detection_msg->landmarks) {
    // iterate through the gates to see which one is closest to the pose
    for(Gate gate : gate_centers_) {
      double dist = alphapilot::dist(detection.pose.position, gate.position.position);
      pq.push(std::tie(detection.gate, gate.id, dist));
    }
  }
  // iterate through and assign lowest distance matching
  std::unordered_set<std::string> detected_ids;
  std::unordered_set<std::string> actual_ids;
  std::map<std::string, std::string> detected_to_actual;
  while(!pq.empty() && detected_ids.size() < detection_msg->landmarks.size()) {
    auto top = pq.top();
    std::string detected = std::get<0>(top);
    std::string actual = std::get<1>(top);
    if(detected_ids.count(detected) == 0 && actual_ids.count(actual) == 0) {
      detected_ids.insert(detected);
      actual_ids.insert(actual);
      detected_to_actual[detected] = actual;
    }
    pq.pop();
  }
  // set the ids to be what was calculated earlier
  for(auto it = detection_msg->landmarks.begin(); it != detection_msg->landmarks.end(); it++) {
    it->gate = detected_to_actual[it->gate];
  }
}

void FactorGraphEstimator::group_gates() {
  auto landmarks = get_landmark_positions();
  std::map<int, std::list<Landmark>> gates = {};
  int num_gates = gates.size();

  for (const auto& landmark : landmarks) {
    std::string l_id = landmark.id;
    std::string l_type = landmark.type;

    // Convert to Landmark
    Landmark new_landmark;
    new_landmark.id = l_id;
    new_landmark.type = l_type;
    new_landmark.position.x = landmark.position.x;
    new_landmark.position.y = landmark.position.y;
    new_landmark.position.z = landmark.position.z;
    // Separate airr logos into top and bottom airr logos
    if (new_landmark.type == "7" && new_landmark.position.z < 1.5) {
      new_landmark.type = "8";
    }
    bool child_of_gate = false;
    for (auto gate : gates) {
      // This landmark type already exists in this gate
      if (object_in_gate(gate.second, l_type)) {
        continue;
      } else if (object_close_to_gate(gate.second, new_landmark)) { // Checks if the landmark is close enough to the subfeatures in the gate
        // Add landmark to gate
        gate.second.push_back(new_landmark);
        child_of_gate = true;
        // No need to check against other gates so exit
        break;
      }
    }
    // If this landmark wasn't added to a gate, create a new gate
    if (!child_of_gate) {
      num_gates++; // Increment the number of gates
      std::list<Landmark> new_gate;
      new_gate.push_back(new_landmark); // add this landmark to the new gate
      gates[num_gates] = new_gate;
    }
  }
}

bool FactorGraphEstimator::object_in_gate(std::list<Landmark> gate, std::string l_type) {
  for (auto subfeature : gate) {
    // Subfeature type already exists in this gate
    if (subfeature.type == l_type) {
      return true;
    }
  }
  return false;
}

bool FactorGraphEstimator::object_close_to_gate(std::list<Landmark> gate, Landmark l) {
  double tolerance = 0.1; //tolerance in m
  for (auto subfeature : gate) {
    double expected_dist = subfeature_dist(subfeature.type, l.type);
    double actual_dist = alphapilot::dist(subfeature, l);
    if (abs(actual_dist - expected_dist) < tolerance) {
      return true;
    }
  }
  return false;
}

/**
 * Adds the local factor graph that was constructed during the most recent
 * callbacks, then clears the graph for the next iteration.
 * This optimizes the entire state trajectory
 */
void FactorGraphEstimator::run_optimize() {
  // TODO make a boolean and return false if we fail to lock
  std::unique_lock<std::mutex> opt_lck_guard(optimization_lck_, std::try_to_lock);
  if(!opt_lck_guard.owns_lock()) {
    std::cout << "ERROR: trying to have multiple optimziations of factor graph running at the same time" << std::endl;
    return;
  }
  if (current_incremental_graph_->empty()) {
    //std::cout << "Warning: cannot optimize over a empty graph" << std::endl;
    optimization_lck_.unlock();
    return;
  }

  // run the optimization and output the final state
  std::unique_lock<std::mutex> graph_lck_guard(graph_lck_);
  std::unique_lock<std::mutex> latest_state_lck_guard(latest_state_lck_);
  int temp_index = index_;
  int temp_bias_index = bias_index_;
  drone_state previous_state = current_pose_estimate_;
  Pose3 previous_guess_pose = current_position_guess_;
  if(debug_) {
    std::cout << "\nstarting optimization at index = " << temp_index << std::endl;
  }
  latest_state_lck_guard.unlock();

  std::shared_ptr<gtsam::NonlinearFactorGraph> graph_copy = current_incremental_graph_;
  std::shared_ptr<gtsam::Values> guesses_copy = gtsam_current_state_initial_guess_;

  // clear incremental graph and current guesses
  current_incremental_graph_ = std::make_shared<gtsam::NonlinearFactorGraph>();
  gtsam_current_state_initial_guess_ = std::make_shared<gtsam::Values>();

  graph_lck_guard.unlock();

  if (debug_) {
    history_.insert(*guesses_copy);
    std::cout << "\nguess state guesses";
    KeyFormatter keyFormatter = gtsam::DefaultKeyFormatter;
    std::cout << "\nValues with " << guesses_copy->size() << " values:" << std::endl;
    for(auto key = guesses_copy->begin(); key != guesses_copy->end(); key++) {
      cout << "Value (" << keyFormatter(key->key) << "): ";
      Symbol symbol = Symbol(key->key);
      if(symbol.chr() == 'x') {
        Pose3 temp = key->value.cast<Pose3>();
        std::cout << temp << "\n";
      } else if(symbol.chr() == 'v') {
        Vector3 temp = key->value.cast<Vector3>();
        std::cout << temp.transpose() << "\n";
      } else if(symbol.chr() == 'l' || symbol.chr() == 'a') {
        Point3 temp = key->value.cast<Point3>();
        std::cout << temp.transpose() << "\n";
      } else if(symbol.chr() == 'b') {
        gtsam::imuBias::ConstantBias temp = key->value.cast<imuBias::ConstantBias>();
        std::cout << temp << "\n";
      }
    }
    std::cout << "\nincremental graph with error" << std::endl;
    graph_copy->printErrors(history_);
  }

  auto start = std::chrono::steady_clock::now();

  // run update and run optimization
  ISAM2Result results = isam_.update(*graph_copy, *guesses_copy);
  // print out optimizations statistics
  optimization_stats_.variablesReeliminated = results.variablesReeliminated;
  optimization_stats_.variablesRelinearized = results.variablesRelinearized;
  if(results.errorBefore) {
    optimization_stats_.errorBefore = *results.errorBefore;
  } else {
    optimization_stats_.errorBefore = -1;
  }
  if(results.errorAfter) {
    optimization_stats_.errorAfter = *results.errorAfter;
  } else {
    optimization_stats_.errorAfter = -1;
  }

  // TODO should I calculate the entire solutions or just single ones
  // set the guesses of state to the correct output, update to account for changes since last optimization
  // TODO use covariance
  Pose3 local_position_optimized = isam_.calculateEstimate<Pose3>(symbol_shorthand::X(temp_index));
  Matrix pose_cov = isam_.marginalCovariance(symbol_shorthand::X(temp_index));
  Vector3 local_velocity_optimized = isam_.calculateEstimate<Vector3>(symbol_shorthand::V(temp_index));
  Matrix vel_cov = isam_.marginalCovariance(symbol_shorthand::V(temp_index));
  if (use_imu_bias_ && use_imu_factors_) {
    current_bias_guess_ = isam_.calculateEstimate<imuBias::ConstantBias>(symbol_shorthand::B(temp_bias_index));
  }

  std::unique_lock<std::mutex> landmark_lck_guard(landmark_lck_);
  // calculate the locations of the landmarks
  landmark_locations_.clear();
  for(int i = 0; i < gate_landmark_index_; i++) {
    Point3 landmark = isam_.calculateEstimate<Point3>(symbol_shorthand::L(i));
    alphapilot::Landmark new_landmark;
    new_landmark.position.x = landmark.x();
    new_landmark.position.y = landmark.y();
    new_landmark.position.z = landmark.z();
    std::string typeAndId = id_to_landmark_map_[i];
    new_landmark.id = typeAndId.substr(0, typeAndId.find("_"));
    new_landmark.type = typeAndId.substr(typeAndId.find("_")+1, std::string::npos);
    /*
    if(debug_) {
      std::cout << "id = " << i << " " << new_landmark << "\n";
    }
     */
    landmark_locations_.insert(std::make_pair(i, new_landmark));
    // if it is a center of a gate store it in that vector directly
  }
  landmark_lck_guard.unlock();
  calculate_gate_centers();

  // calculate the centers of aruco
  if(debug_) {
    std::cout << "\nARUCO POSITIONS\n" << std::endl;
  }
  aruco_locations_lck_.lock();
  aruco_locations_.clear();
  aruco_locations_.reserve(aruco_indexes_.size() * 4);
  for(int id_base : aruco_indexes_) {
    Vector3 aruco;
    aruco << 0, 0, 0;
    int num_points = 0;
    for(int i = 0; i < 4; i++) {
      if (isam_.valueExists(symbol_shorthand::A(id_base + i))) {
        Point3 temp = isam_.calculateEstimate<Point3>(symbol_shorthand::A(id_base + i));
        std::cout << "\033[1;31mAruco " << id_base + i << ": ";
        //print_projection(temp_index, temp, camera_map["left_right"], Point2());
        std::cout << "\033[0m" << std::endl;
        alphapilot::Point pos;
        pos.x = temp.x();
        pos.y = temp.y();
        pos.z = temp.z();
        aruco_locations_.push_back(pos);
        aruco += temp.vector();
        num_points++;
      } else {
        std::cout << "Aruco " << id_base << i << " does not exist in isam" << std::endl;
      }
    }
    aruco /= num_points;
    if(debug_) {
      std::cout << "Aruco with id " << (id_base - 1) / 10 << ": " << aruco.transpose() << std::endl;
    }
  }
  aruco_locations_lck_.unlock();

  // get timing statistics
  ++optimization_stats_.optimizationIterations;
  double optimization_iterations = optimization_stats_.optimizationIterations;
  auto end = std::chrono::steady_clock::now();
  std::chrono::duration<double> diff = end - start;
  double optimization_time = diff.count();
  optimization_stats_.optimizationTimeAvg = (optimization_iterations - 1.0)/optimization_iterations*optimization_stats_.optimizationTimeAvg +
          optimization_time/optimization_iterations;
  optimization_stats_.optimizationTime = optimization_time;

  if(debug_) {
    std::cout << optimization_stats_ << std::endl;
  }

  graph_lck_guard.lock();
  latest_state_lck_guard.lock();

  // get the current state
  drone_state current_state = current_pose_estimate_;

  /*
  if(debug_) {
    std::cout << "\ntemp_index = " << temp_index << "\n"
              << "Position\n" << local_position_optimized << "\n"
              << "Velocity: " << local_velocity_optimized.transpose() << "\n"
              << "Current State: " << current_state << "\n"
              << "Bias guess: " << current_bias_guess_ << std::endl;
  }
   */
  if (debug_) {
    std::cout << "Bias Guess: " << current_bias_guess_ << std::endl;
  }

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
  /*
  if (debug_) {
    std::cout << "update pos = " << update << "\n";
    std::cout << "update vel = " <<  (gtsam::Vector3(current_state.x_dot, current_state.y_dot, current_state.z_dot)
          - gtsam::Vector3(previous_state.x_dot, previous_state.y_dot, previous_state.z_dot)).transpose() << std::endl;
  }
   */

  trans_update = Point3(previous_state.x + trans_update.x(),
          previous_state.y + trans_update.y(), previous_state.z + trans_update.z());
  // TODO clean up
  current_position_guess_ = previous_guess_pose * update;
  current_position_guess_ = Pose3(current_position_guess_.rotation(), trans_update);

  current_velocity_guess_ = local_velocity_optimized + gtsam::Vector3(current_state.x_dot, current_state.y_dot, current_state.z_dot)
          - gtsam::Vector3(previous_state.x_dot, previous_state.y_dot, previous_state.z_dot);

  if(debug_) {
    std::cout << "\ncurrent position guess\n" << current_position_guess_ << "\n";
    std::cout << "\ncurrent velocity guess = " << current_velocity_guess_.transpose() << "\n\n\n" << std::endl;
  }

  graph_lck_guard.unlock();

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
}

/**
 * callback for the imu, integrates the reading using the preintegrator_imu_, and then
 * calls propagate_imu to update the estimated state given to GTSAM
 * @param imu_data, assumes the accel and angular vel was over the last dt seconds
 */
void FactorGraphEstimator::callback_imu(const std::shared_ptr<IMU_readings> imu_data) {
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
  current_pose_estimate_.time = last_imu_time_;
  if (dt <= 0 || dt > 10) {
    std::cout << "ERROR: cannot use imu reading with dt <= 0 || dt > 10" << std::endl;
    return;
  }
  if((std::fabs(imu_data->x_accel) > 50 || std::fabs(imu_data->y_accel) > 50 || std::fabs(imu_data->z_accel) > 50) ||
     (std::fabs(imu_data->x_accel) < 1e-6 && std::fabs(imu_data->y_accel) < 1e-6 && std::fabs(imu_data->z_accel) < 1e-6)) {
    std::cout << "ignoring IMU since it it too large or all zero" << std::endl;
    return;
  }
  // -1 if invert, 1 if !invert
  double invert_x = invert_x_ ? -1.0 : 1.0;
  double invert_y = invert_y_ ? -1.0 : 1.0;
  double invert_z = invert_z_ ? -1.0 : 1.0;

  imu_data->x_accel = 0;
  imu_data->y_accel = 0;
  imu_data->z_accel = 9.81;
  imu_data->roll_vel = 0.0;
  imu_data->pitch_vel = 0.0;
  imu_data->yaw_vel = 0.0;

  // updates the current guesses of position
  Vector3 accel = Vector3(imu_data->x_accel * invert_x, imu_data->y_accel * invert_y, imu_data->z_accel * invert_z);
  Vector3
          ang_rate = Vector3(imu_data->roll_vel * invert_x, imu_data->pitch_vel * invert_y, imu_data->yaw_vel * invert_z);
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
    propagate_imu(accel, ang_rate, dt);
    position_update_ = true;
  }
  //std::cout << __LINE__ << std::endl;
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
  double c_phi, c_theta, c_psi, s_phi, s_theta, s_psi;
  double ux, uy, uz;
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
  // overflow of the values are handled by the gtsam constructor
  double r_result, p_result, y_result;
  r_result = roll + (roll_dot + (s_phi * s_theta / c_theta) * pitch_dot + (c_phi * s_theta / c_theta) * yaw_dot) * dt;
  p_result = pitch + (c_phi * pitch_dot - s_phi * yaw_dot) * dt;
  y_result = yaw + (s_phi / c_theta * pitch_dot + c_phi / c_theta * yaw_dot) * dt;

  // apply the update
  // the ordering is correct for gtsam
  current_position_guess_ = Pose3(Rot3::Ypr(y_result, p_result, r_result),
                                  Point3(result.x, result.y, result.z));
  // if (debug_) {
  //   std::cout << current_position_guess_.rotation().rpy()[0] << std::endl;
  // }
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

/**
 * Creates a new state variable by increasing the state index_
 * and adds it to the graph.
 * The basic process for doing so is creating a factor relating
 * previous state variables to current state variables (aka. motion model)
 * and then adding a guess for the current values of those states
 * to give the optimizer a value to start from
 */
void FactorGraphEstimator::add_imu_factor() {
  if (!position_update_) {
    std::cout << "Position has not been update, not adding IMU factor" << std::endl;
    return;
  }

  // Update bias at a slower rate
  lock_guard<mutex> lck(graph_lck_);
  if (use_imu_bias_ && index_ % imu_bias_incr_ == 0) {
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
  lock_guard<mutex> preintegrator_lck(preintegrator_lck_);
  // motion model of position and velocity variables
  if(debug_) {
    std::cout << "creating IMU index at = " << index_ << " to " << index_ + 1 << std::endl;
  }

  if(use_imu_prop_) {
    lock_guard<mutex> latest_state_lck(latest_state_lck_);
    // Add factor to graph and add initial variable guesses
    gtsam_current_state_initial_guess_->insert(symbol_shorthand::X(index_ + 1),
                                               current_position_guess_);
    gtsam_current_state_initial_guess_->insert(symbol_shorthand::V(index_ + 1),
                                               current_velocity_guess_);
  }

  ImuFactor imufac(symbol_shorthand::X(index_),
                   symbol_shorthand::V(index_),
                   symbol_shorthand::X(index_ + 1),
                   symbol_shorthand::V(index_ + 1),
                   symbol_shorthand::B(bias_index_),
                   preintegrator_imu_);

  // prevents current_vel and pos from being updated
  current_incremental_graph_->add(imufac);

  // clear IMU rolling integration
  preintegrator_imu_.resetIntegration();
  position_update_ = false;
}

// Assumes that there is only a single instance of an ID per camera
void FactorGraphEstimator::aruco_callback(const std::shared_ptr<alphapilot::ArucoDetections> msg) {
  if(!use_aruco_factors_) {
    return;
  }

  if (camera_map.find(msg->camera) == camera_map.end()) {
    std::cout << "ERROR using invalid camera name " << msg->camera << " make sure to register a camera first"
              << std::endl;
    return;
  }
  gtsam_camera camera = camera_map[msg->camera];
  int image_index = find_camera_index(msg->time);
  // TODO there is some time delay in aruco, check to make sure we have the right image
  if(image_index == -1) {
    std::cout << "ERROR: invalid index: -1 for the camera on aruco callback time: "
              << msg->time << " candidates: " << time_map_.size() << std::endl;
    return;
  }

  lock_guard<mutex> graph_lck(graph_lck_);

  std::cout << "iterating through detecitons" << std::endl;

  // list of ids that have been added so there are not duplicates
  for(const alphapilot::ArucoDetection& detection : msg->detections) {
    int id_base = detection.id * 10 + 1;


    // use the first detection to init the location of the aruco marker in world frame
    if(aruco_indexes_.find(id_base) == aruco_indexes_.end()) {
      Point3 location = Point3(detection.pose.position.x, detection.pose.position.y, detection.pose.position.z);
      // use the relative distance to get estimate of aruco position
      //double dist_aruco = alphapilot::dist(detection.pose.position, alphapilot::Point());
      //gtsam::PinholeCamera<Cal3_S2> gt_cam(camera.transform, *camera.K);
      //auto projection_thing = gt_cam.backproject(Point2(detection.points[0].x, detection.points[0].y), dist_aruco);
      //Point3 prior = current_position_guess_ * projection_thing;

      // convert drone frame to global frame
      Point3 prior = current_position_guess_ * location;
      if(debug_) {
        std::cout << "creating aruco with id " << id_base << " at image index " << image_index << ": "
                  << location << std::endl;
      }
      // convert drone frame to gloal frame
      // insert a prior and state for each aruco
      for(unsigned int i = 0; i < detection.points.size(); i++) {
        Point3 tmp = prior;
        if (i == 0) {
          tmp += Point3(-0.1, -0.1, 0);
        } else if (i == 1) {
          tmp += Point3(-0.1, 0.1, 0);
        } else if (i == 2) {
          tmp += Point3(0.1, 0.1, 0);
        } else if (i == 3) {
          tmp += Point3(0.1, -0.1, 0);
        }
        // prior += Point3(0, 0, 0.2);
        gtsam_current_state_initial_guess_->insert(symbol_shorthand::A(id_base + i), tmp);
        // TODO use the orientation to create a new position for each
        current_incremental_graph_->add(PriorFactor<Point3>(symbol_shorthand::A(id_base + i),
                                                            tmp,
                                                            aruco_pose_prior_noise_));
      }
      // create between factors for each corresponding set of points
      // tl to tr
      gtsam::noiseModel::Diagonal::shared_ptr aruco_constraint_noise = noiseModel::Isotropic::Sigma(1, 0.05);
      current_incremental_graph_->emplace_shared<RangeFactor<Point3>>(symbol_shorthand::A(id_base),
                                                symbol_shorthand::A(id_base + 1),
                                                0.2,
                                                aruco_constraint_noise);

      current_incremental_graph_->emplace_shared<RangeFactor<Point3>>(symbol_shorthand::A(id_base + 1),
                                                                      symbol_shorthand::A(id_base + 2),
                                                                      0.2,
                                                                      aruco_constraint_noise);

      current_incremental_graph_->emplace_shared<RangeFactor<Point3>>(symbol_shorthand::A(id_base + 2),
                                                                      symbol_shorthand::A(id_base + 3),
                                                                      0.2,
                                                                      aruco_constraint_noise);

      current_incremental_graph_->emplace_shared<RangeFactor<Point3>>(symbol_shorthand::A(id_base),
                                                                      symbol_shorthand::A(id_base + 2),
                                                                      0.28,
                                                                      aruco_constraint_noise);
      current_incremental_graph_->emplace_shared<RangeFactor<Point3>>(symbol_shorthand::A(id_base + 1),
                                                                      symbol_shorthand::A(id_base + 3),
                                                                      0.28,
                                                                      aruco_constraint_noise);
      aruco_indexes_.insert(id_base);
    }

    for(unsigned int i = 0; i < detection.points.size(); i++) {
      GenericProjectionFactor<Pose3, Point3, Cal3_S2>::shared_ptr projectionFactor;
      Point2 detection_coords = Point2(detection.points.at(i).x, detection.points.at(i).y);
      projectionFactor = boost::make_shared<GenericProjectionFactor<Pose3, Point3, Cal3_S2>>(
              detection_coords, aruco_camera_noise_, symbol_shorthand::X(image_index),
              symbol_shorthand::A(id_base + i), camera.K, camera.transform);
      current_incremental_graph_->push_back(projectionFactor);
      // if(debug_) {
      //   // TODO generalize
      //   Point3 point = Point3(3.0, 2.2, 1);
      //   print_projection(image_index, point, camera, detection_coords);
      // }
    }
  }

}

void FactorGraphEstimator::add_projection_prior(std::shared_ptr<Landmark> msg) {
  Point3 prior = Point3(msg->position.x, msg->position.y, msg->position.z);
  std::string object_name = msg->id + "_" + msg->type;
  id_to_landmark_map_.insert(std::make_pair(gate_landmark_index_, object_name));
  landmark_to_id_map_.insert(std::make_pair(object_name, gate_landmark_index_));
  if(debug_) {
    std::cout << "\nadding object prior " << object_name << " with index = " << gate_landmark_index_ << "\n"
              << "Prior: " << prior.transpose() << "\n"
              << "Noise: " << landmark_prior_noise_->sigmas().transpose() << std::endl;
  }
  // add prior factor
  current_incremental_graph_->add(PriorFactor<Point3>(symbol_shorthand::L(gate_landmark_index_),
                                                prior,
                                                landmark_prior_noise_));
  // add initial guess as well
  gtsam_current_state_initial_guess_->insert(symbol_shorthand::L(gate_landmark_index_), prior);
  ++gate_landmark_index_;
}

/**
 * Takes in detected landmarks from image space and adds them to the factor
 *  graph, uses the timestamp on the message to determine what state the image corresponds to
 *
 * Optimizes the enitre state trajecotry at the end
 *
 * @param map<landmark_id, uv_coords> landmark_data
 */
void FactorGraphEstimator::callback_cm(const std::shared_ptr<GateDetections> landmark_data) {
  std::cout << __func__ << std::endl;

  if (!use_camera_factors_) {
    return;
  }

  // if there is no position updates this is unconstrained
  if (camera_map.find(landmark_data->camera_name) == camera_map.end()) {
    std::cout << "ERROR using invalid camera name " << landmark_data->camera_name << " make sure to register a camera first"
              << std::endl;
  }

  int image_index = find_camera_index(landmark_data->time);
  if(image_index == -1) {
    std::cout << "ERROR: invalid index: -1 for the camera on detection callback" << std::endl;
    return;
  }

  // TODO add parameter to enabling or disabling this
  // groups the gates and sets the ids correctly
  if(reassign_gate_ids_) {
    assign_gate_ids(landmark_data);
  }

  for (const auto &seen_gate : landmark_data->landmarks) {
    for(const auto &subfeature : seen_gate.subfeatures) {
      std::string l_id = seen_gate.gate+"_"+subfeature.type;
      std::string object_type = subfeature.type;

      // New Landmark - Add a new Factor
      gtsam_camera camera = camera_map[landmark_data->camera_name];

      Point2 detection_coords(subfeature.x, subfeature.y);

      int id = -1;
      if(landmark_to_id_map_.find(l_id) != landmark_to_id_map_.end()) {
        id = landmark_to_id_map_[l_id];
      } else {
        // TODO if fails to find and index that matches, handle
        std::cout << "\n\nERROR: invalid subfeature gate: " << seen_gate.gate << subfeature
                  << "\nmust register landmark first\n\n" << std::endl;
        return;
      }
      // TODO reimplement
      /*
      if(debug_) {
        alphapilot::Landmark l = landmark_locations_[id];
      }
      */
      GenericProjectionFactor<Pose3, Point3, Cal3_S2>::shared_ptr projectionFactor;
      if (object_noises_.find(object_type) != object_noises_.end()) {
        projectionFactor = boost::make_shared<GenericProjectionFactor<Pose3, Point3, Cal3_S2>>(
            detection_coords, object_noises_[object_type], symbol_shorthand::X(image_index),
            symbol_shorthand::L(id), camera.K, camera.transform);
      } else {
        projectionFactor = boost::make_shared<GenericProjectionFactor<Pose3, Point3, Cal3_S2>>(
            detection_coords, default_camera_noise_, symbol_shorthand::X(image_index),
            symbol_shorthand::L(id), camera.K, camera.transform);
      }

      graph_lck_.lock();
      current_incremental_graph_->push_back(projectionFactor);
      graph_lck_.unlock();
    }

  }
}

/**
 * Takes in a global estimate of pose, finds the difference and accumulates the delta for the factor
 * @param odom_data
 */
void FactorGraphEstimator::callback_odometry(const std::shared_ptr<drone_state> odom_data) {
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
  gtsam_current_state_initial_guess_->insert(symbol_shorthand::X(index_ + 1),
                                            Pose3(new_rot, new_point));

  Vector3 temp_vel = current_velocity_guess_ + vel_change_accum_;
  gtsam_current_state_initial_guess_->insert(symbol_shorthand::V(index_ + 1), temp_vel);

  // add constraint on the poses
  // transforms the position offset into body frame offset
  Point3 body_trans = current_position_guess_.transformTo(pose_trans_accum_ + current_position_guess_.translation());


  // assumes that the pose change is in body frame
  current_incremental_graph_->emplace_shared<BetweenFactor<Pose3>>(symbol_shorthand::X(index_),
                                                                  symbol_shorthand::X(index_ + 1),
                                                                  Pose3(pose_rot_accum_, body_trans),
                                                                  odometry_pose_noise_);

  // add constraint on the velocity
  current_incremental_graph_->emplace_shared<BetweenFactor<Vector3>>(symbol_shorthand::V(index_),
                                                                    symbol_shorthand::V(index_ + 1),
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

void FactorGraphEstimator::callback_range(int rangestuff) {
  if (!use_range_factors_) {
    return;
  }
}


// under no locks
void FactorGraphEstimator::add_priors(const drone_state &initial_state) {

  current_position_guess_ =
      Pose3(Rot3::Quaternion(initial_state.qw, initial_state.qx, initial_state.qy, initial_state.qz),
            Point3(initial_state.x, initial_state.y, initial_state.z));
  current_velocity_guess_ = Vector3(initial_state.x_dot, initial_state.y_dot, initial_state.z_dot);

  // init the set of Values passed to GTSAM during optimization
  vel_change_accum_ = Vector3(0, 0, 0);

  // insert initial guesses of state
  gtsam_current_state_initial_guess_->insert(symbol_shorthand::X(index_),
                                            current_position_guess_);
  gtsam_current_state_initial_guess_->insert(symbol_shorthand::V(index_),
                                            current_velocity_guess_);

  if (use_imu_factors_) {
    gtsam_current_state_initial_guess_->insert(symbol_shorthand::B(bias_index_),
                                               current_bias_guess_);
  }

  // Assemble prior noise model and add it the graph.
  std::vector<double> pose_noise_arr = prior_config_.initial_pose_noise;
  noiseModel::Diagonal::shared_ptr pose_noise_model = noiseModel::Diagonal::Sigmas((Vector(6)
      << pose_noise_arr[0], pose_noise_arr[1], pose_noise_arr[2], pose_noise_arr[3], pose_noise_arr[4], pose_noise_arr[5]).finished()); // rad,rad,rad,m, m, m
  if (debug_) {
    std::cout << "prior pose noise = " << pose_noise_model->sigmas().transpose() << std::endl;
  }

  noiseModel::Diagonal::shared_ptr
      velocity_noise_model = noiseModel::Isotropic::Sigma(3, prior_config_.initial_vel_noise);
  if (debug_) {
    std::cout << "prior vel noise = " << velocity_noise_model->sigmas().transpose() << std::endl;
  }

  noiseModel::Diagonal::shared_ptr bias_noise_model = noiseModel::Isotropic::Sigma(6, prior_config_.initial_bias_noise);
  if (debug_) {
    std::cout << "bias_noise_model prior = " << bias_noise_model->sigmas().transpose() << std::endl;
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

  //run_optimize();
}

drone_state FactorGraphEstimator::latest_state(bool optimize) {
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

void FactorGraphEstimator::register_camera(const std::string name,
                                           const std::shared_ptr<gtsam::Point3> translation,
                                           const std::shared_ptr<gtsam::Rot3> rotation,
                                           const std::shared_ptr<camera_info> camera_info) {
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
  cam.transform = Pose3(*rotation, *translation);


  std::cout << "Registered camera:" << "\n";
  cam.K->print();
  std::cout << "\ntransform = " << cam.transform << std::endl;
  camera_map.insert(std::pair<std::string, gtsam_camera>(name, cam));
}

optimization_stats FactorGraphEstimator::get_optimization_stats() {
  return optimization_stats_;
}

int FactorGraphEstimator::find_camera_index(double time) {
  if(debug_) {
    std::cout << std::setprecision(15) << "finding state close to " << time << std::endl;
  }
  // if negative dt
  if(time_map_.empty()) {
    std::cout << "ERROR: time map is empty cannot match camera frame" << std::endl;
    return -1;
  }
  if(time_map_.rbegin()->second + pairing_threshold_ < time) {
    std::cout << "ERROR: time " << time << " is much larger than " << time_map_.rbegin()->second + pairing_threshold_ << "no correspondence" << std::endl;
    return -1;
  }
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

void FactorGraphEstimator::print_projection(int image_index, Point3 location, gtsam_camera camera, Point2 detection_coords) {
  // TODO throws exception
  Pose3 position;
  bool print = false;
  if(history_.exists(symbol_shorthand::X(image_index))) {
    print = true;
    position = history_.at<Pose3>(symbol_shorthand::X(image_index));
  }
  if(!print && gtsam_current_state_initial_guess_->exists(symbol_shorthand::X(image_index))) {
    print = true;
    position = gtsam_current_state_initial_guess_->at<Pose3>(symbol_shorthand::X(image_index));
  }
  if(print) {
    //std::cout << "\n\nposition " << position << std::endl;
    //std::cout << "\ncamera transform " << camera.transform << std::endl;
    position = position.compose(camera.transform);
    std::cout << "\nCamera World Position: " << position << std::endl;
    PinholeCamera<Cal3_S2> test_cam(position, *camera.K);
    Point2 measurement = test_cam.project(location);
    std::cout << "Location: " << location << "\n";
    std::cout << "Detection got: " << detection_coords << "\n"
              << "Expected     : " << measurement<< std::endl;

  }
}

std::vector<alphapilot::Gate> FactorGraphEstimator::get_gates() {
  lock_guard<mutex> gate_lck(gate_lck_);
  return gate_centers_;
}

// also under landmark_lck
void FactorGraphEstimator::calculate_gate_centers() {
  lock_guard<mutex> gate_lck(gate_lck_);
  lock_guard<mutex> landmark_lck(landmark_lck_);
  // add all the positions together
  std::map<std::string, Vector4> map;
  for(auto & landmark_location : landmark_locations_) {
    std::string gate_id = landmark_location.second.id;
    if(map.count(gate_id) == 0) {
      Vector4 temp;
      temp << 0, 0, 0, 0;
      map[gate_id] = temp;
    }
    Vector4 temp;
    temp << landmark_location.second.position.x, landmark_location.second.position.y, landmark_location.second.position.z, 1;
    map[gate_id] += temp;
  }
  for(auto & it : map) {
    Gate gate;
    gate.id = it.first;
    // find the average position
    it.second /= it.second(3);
    // set the postion of each gate
    gate.position.position.x = it.second(0);
    gate.position.position.y = it.second(1);
    gate.position.position.z = it.second(2);
    gate_centers_.push_back(gate);
  }
}

std::vector<alphapilot::Point> FactorGraphEstimator::get_aruco_locations() {
  lock_guard<mutex> aruco_locations_lck(aruco_locations_lck_);
  return aruco_locations_;
}

} // estimator
} // StateEstimator
