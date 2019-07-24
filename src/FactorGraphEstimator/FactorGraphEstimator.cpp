#include <StateEstimator/estimation/FactorGraph/FactorGraphEstimator.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>

#include <yaml-cpp/yaml.h>
#include <alphapilot_common/GateConstraints.h>

using namespace std;
using namespace gtsam;

namespace alphapilot {
namespace estimator {
/**
 * Constructor for the Factor Graph Estimator; sets up the noise models
 * @param initial_state, start location to begin the optimization
 */
FactorGraphEstimator::FactorGraphEstimator(const estimator_config &estimator_config) {
  // TODO not supported start anymore
  debug_ = estimator_config.debug;
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

  // pose factor noise
  use_pose_factors_ = estimator_config.poseFactorParams.usePoseFactor;
  std::vector<double> odom_vel_arr = estimator_config.poseFactorParams.poseVelNoise;
  Vector3 odom_vel_noise = Vector3(odom_vel_arr[0], odom_vel_arr[1], odom_vel_arr[2]);
  if(debug_) {
    std::cout << "odom vel noise = " << odom_vel_noise.transpose() << std::endl;
  }
  odometry_vel_noise_ = noiseModel::Diagonal::Sigmas(odom_vel_noise);
  Vector6 pose_noise;
  std::vector<double> pose_noise_arr = estimator_config.poseFactorParams.poseNoise;
  pose_noise
      << pose_noise_arr[0], pose_noise_arr[1], pose_noise_arr[2], pose_noise_arr[3], pose_noise_arr[4], pose_noise_arr[5];
  if(debug_) {
    std::cout << "pose noise = " << pose_noise.transpose() << std::endl;
  }
  odometry_pose_noise_ = noiseModel::Diagonal::Sigmas(pose_noise);

  // add the priors on state
  add_priors(estimator_config.priorConfig.initial_state);
}

FactorGraphEstimator::FactorGraphEstimator(const std::string &config_file, const std::string &full_path) {

  YAML::Node config = YAML::LoadFile(config_file);

  if(debug_) {
    std::cout << "loading config file from " << config_file << std::endl;
  }

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
                                                                     {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
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
      got_detections_from_[camera_name] = false;
    }

    std::vector<std::string> object_names = alphapilot::get<std::vector<std::string>>("objects", camera_config, {});
    for (auto &object_name : object_names) {
      double noise = alphapilot::get<double>(object_name + "_noise", camera_config, default_noise);
      gtsam::noiseModel::Diagonal::shared_ptr
          noise_model = noiseModel::Isotropic::Sigma(2, noise);  // one pixel in u and v
      object_noises_.insert(std::pair<std::string, gtsam::noiseModel::Diagonal::shared_ptr>(object_name, noise_model));
    }
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

  // add the priors on state
  add_priors(prior_config_.initial_state);
}

std::map<std::string, std::array<double, 3>> FactorGraphEstimator::get_landmark_positions() {
  // TODO return variable, set up elsewhere when optimize is called
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

std::map<int, std::list<Landmark>> FactorGraphEstimator::group_gates() {
  auto landmarks = get_landmark_positions();
  std::map<int, std::list<Landmark>> gates = {};
  int num_gates = gates.size();

  for (auto landmark : landmarks) {
    std::string l_id = landmark.first;
    std::string l_type = l_id.substr(0, l_id.find('-'));

    // Convert to Landmark
    Landmark new_landmark;
    new_landmark.id = l_id;
    new_landmark.type = l_type;
    new_landmark.position.x = landmark.second[0];
    new_landmark.position.y = landmark.second[1];
    new_landmark.position.z = landmark.second[2];
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
  return gates;
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
  if(!optimization_lck_.try_lock()) {
    return;
  }
  if (current_incremental_graph_->empty()) {
    //std::cerr << "ERROR: cannot optimize over a empty graph" << std::endl;
    optimization_lck_.unlock();
    return;
  }

  // run the optimization and output the final state
  graph_lck_.lock();
  latest_state_lck_.lock();
  if(debug_) {
    std::cout << "\nstarting optimization at index = " << index_ << std::endl;
  }
  int temp_index = index_;
  drone_state previous_state = current_pose_estimate_;
  Pose3 previous_guess_pose = current_position_guess_;
  latest_state_lck_.unlock();

  std::shared_ptr<gtsam::NonlinearFactorGraph> graph_copy = current_incremental_graph_;
  std::shared_ptr<gtsam::Values> guesses_copy = gtsam_current_state_initial_guess_;

  // clear incremental graph and current guesses
  current_incremental_graph_ = std::make_shared<gtsam::NonlinearFactorGraph>();
  gtsam_current_state_initial_guess_ = std::make_shared<gtsam::Values>();

  // add the camera factors
  if(debug_) {
    std::cout << detections_.size() << " held back detections from cameras being added" << std::endl;
  }
  for(const detection& temp : detections_) {
    landmark_factors_[temp.id]->add(temp.point, symbol_shorthand::X(temp.index));
  }
  detections_.clear();

  graph_lck_.unlock();

  if (debug_) {
    history_.insert(*guesses_copy);
    std::cout << "\nguess state guesses" << std::endl;
    KeyFormatter keyFormatter = gtsam::DefaultKeyFormatter;
    std::cout << "Values with " << guesses_copy->size() << " values:" << std::endl;
    for(auto key = guesses_copy->begin(); key != guesses_copy->end(); key++) {
      cout << "\nValue " << keyFormatter(key->key) << ": ";
      Symbol symbol = Symbol(key->key);
      if(symbol.chr() == 'x') {
        Pose3 temp = key->value.cast<Pose3>();
        std::cout << temp << "\n";
      } else if(symbol.chr() == 'v') {
        Vector3 temp = key->value.cast<Vector3>();
        std::cout << temp.transpose() << "\n";
      }
    }
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

  graph_lck_.lock();
  latest_state_lck_.lock();

  // get the current state
  drone_state current_state = current_pose_estimate_;

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
  if (debug_) {
    std::cout << "update pos = " << update << std::endl;
    std::cout << "update vel = " <<  (gtsam::Vector3(current_state.x_dot, current_state.y_dot, current_state.z_dot)
          - gtsam::Vector3(previous_state.x_dot, previous_state.y_dot, previous_state.z_dot)).transpose() << std::endl;
  }


  // set the guesses of state to the correct output, update to account for changes since last optimization
  Pose3 local_position_guess = isam_.calculateEstimate<Pose3>(symbol_shorthand::X(temp_index));
  if(debug_) {
    std::cout << "\nposition before\n" << current_position_guess_ << std::endl;
  }

  trans_update = Point3(previous_state.x + trans_update.x(),
          previous_state.y + trans_update.y(), previous_state.z + trans_update.z());
  // TODO clean up
  //current_position_guess_ = Pose3(Rot3(rot_update * odom_q), trans_update);
  current_position_guess_ = previous_guess_pose * update;
  current_position_guess_ = Pose3(current_position_guess_.rotation(), trans_update);
  if(debug_) {
    std::cout << "\nposition after\n" << current_position_guess_ << std::endl;
  }

  current_velocity_guess_ = isam_.calculateEstimate<Vector3>(symbol_shorthand::V(temp_index));
  if(debug_) {
    std::cout << "\nvelocity before = " << current_velocity_guess_.transpose() << std::endl;
  }
  current_velocity_guess_ += gtsam::Vector3(current_state.x_dot, current_state.y_dot, current_state.z_dot)
          - gtsam::Vector3(previous_state.x_dot, previous_state.y_dot, previous_state.z_dot);
  if(debug_) {
    std::cout << "\nvelocity after = " << current_velocity_guess_.transpose() << std::endl;
  }

  if (debug_) {
    std::cout << "\ncurrent position guess\n" << current_position_guess_ << std::endl;
    std::cout << "\ncurrent velocity guess = " << current_velocity_guess_.transpose() << std::endl;
  }

  graph_lck_.unlock();

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
  //used to add one
  int index_adder = 1;
  // mark camera as gotten detections
  got_detections_from_[camera_name] = true;
  // check if we should add factors here
  bool add_factors = true;
  for(const auto &temp : got_detections_from_) {
    add_factors = add_factors && temp.second;
  }
  if(add_factors) {
    // create the newest state
    this->add_factors();
    // clear what cameras have gotten detections
    for(auto &temp : got_detections_from_) {
      temp.second = false;
    }
    index_adder = 0;
  }

  if (!use_camera_factors_) {
    return;
  }
  for (const auto &seen_landmark : *landmark_data) {
    std::string l_id = seen_landmark.first;
    pair<double, double> im_coords = seen_landmark.second;
    std::string object_type = l_id.substr(0, l_id.find('_'));

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
    detection.index = index_ + index_adder;
    detection.point = detection_coords;
    detection.id = l_id;
    detections_.push_back(detection);
    graph_lck_.unlock();
  }
}



void FactorGraphEstimator::callback_range(int rangestuff) {
  if (!use_range_factors_) {
    return;
  }

}

// under no locks
void FactorGraphEstimator::add_priors(const drone_state &initial_state) {
  gtsam_current_state_initial_guess_ = std::make_shared<gtsam::Values>();
  current_incremental_graph_ = std::make_shared<gtsam::NonlinearFactorGraph>();

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
  // priors match initial guess
  current_incremental_graph_->add(PriorFactor<Pose3>(symbol_shorthand::X(index_),
                                                    current_position_guess_,
                                                    pose_noise_model));
  current_incremental_graph_->add(PriorFactor<Vector3>(symbol_shorthand::V(index_),
                                                      current_velocity_guess_,
                                                      velocity_noise_model));

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
    lock_guard<mutex> pose_lck(pose_lck_);
    lock_guard<mutex> graph_lck(graph_lck_);
    lock_guard<mutex> latest_state_lck(latest_state_lck_);

    //clear
    index_ = 0;
    pose_message_count_ = 0;
    current_incremental_graph_ = std::make_shared<gtsam::NonlinearFactorGraph>();
    gtsam_current_state_initial_guess_->clear();
    isam_ = ISAM2(isam_parameters_);
    history_.clear();
  }

  // reset up priors
  add_priors(initial_state);
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

/**
 * Takes in a global estimate of pose, finds the difference and accumulates the delta for the factor
 * @param odom_data
 */
void FactorGraphEstimator::callback_odometry(const std::shared_ptr<drone_state> odom_data) {
  if (!use_pose_factors_) {
    return;
  }
  // otherwise you can have a between factor of two states that do not exist
  position_update_ = true;

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

void FactorGraphEstimator::add_factors() {
  if (!position_update_) {
	std::cout << "position not updated, not adding factors" << std::endl;
    return;
  }
  std::cout << "position updated, adding factors" << std::endl;
  add_pose_factor();
  index_++;
  position_update_ = false;
}

void FactorGraphEstimator::register_camera(const std::string name,
                                           const std::shared_ptr<transform> transform,
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
  cam.transform = Pose3(Rot3::Quaternion(transform->qw, transform->qx, transform->qy, transform->qz),
                        Point3(transform->x, transform->y, transform->z));
  camera_map.insert(std::pair<std::string, gtsam_camera>(name, cam));
}

double FactorGraphEstimator::get_optimization_time() {
  return optimization_time_;
}

} // estimator
} // StateEstimator
