/**
Description goes here and also licesnse?
Authors: Bogdan Vlahov and Jason Gibson
**/
#ifndef FactorGraphEstimator_H_
#define FactorGraphEstimator_H_

#include <gtsam/geometry/SimpleCamera.h>
#include <gtsam/geometry/Quaternion.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <alphapilot_common/Utils.h>

#include <map>
#include <mutex>
#include <memory>
#include <iostream>
#include <cstdio>

#include <yaml-cpp/yaml.h>
#include <unordered_set>

// TODO make callbacks pass by const reference to shared pointer
namespace alphapilot {
namespace estimator {
struct gtsam_camera {
  gtsam::Pose3 transform;
  boost::shared_ptr<gtsam::Cal3_S2> K;
};

struct isam_parameters {
  // https://borg.cc.gatech.edu/sites/edu.borg/html/a00135.html#af5da340f5774c8ccbbdecfc0a5299888
  double relinearizeThreshold = 0.01;
  int relinearizeSkip = 1;
  bool enablePartialRelinearizationCheck = false;
  bool chacheLinearedFactors = false;
  bool enableDetailedResults = true;
  bool findUnusedFactorSlots = false;
  double gaussianWildfireThreshold = 0.001;
};

struct pose_factor_params {
  bool usePoseFactor = true;
  std::vector<double> poseNoise = {0.25, 0.25, 0.25, 0.25, 0.25, 0.25};
  std::vector<double> poseVelNoise = {0.3, 0.3, 0.3};
};

struct camera_factor_params {
  bool useCameraFactor = true;
  double pixelNoise = 10.0;
};

struct prior_config {
  drone_state initial_state;
  double initial_vel_noise = 0.1;
  std::vector<double> initial_pose_noise = {0.05, 0.05, 0.05, 0.25, 0.25, 0.25};//rad, rad, rad, m,m,m
};

struct estimator_config {
  bool debug = false;
  // initial time to start the factor graph at
  double time = 0.0;
  isam_parameters isamParameters;
  pose_factor_params poseFactorParams;
  camera_factor_params cameraFactorParams;
  prior_config priorConfig;
};

struct detection {
  std::string id = "";
  gtsam::Point2 point = gtsam::Point2(-1,-1);
  int index = 0;
};

class FactorGraphEstimator {
 public:
  explicit FactorGraphEstimator(const estimator_config &estimator_config);

#if ENABLE_YAML
  FactorGraphEstimator(const std::string &config_file, const std::string& full_path);
#endif

  virtual void callback_cm(std::shared_ptr<std::map<std::string, std::pair<double, double>>> landmark_data,
                           std::string camera_name);
  // TODO: Figure out what data structure is used for range finders
  virtual void callback_range(const int rangestuff);
  virtual void callback_odometry(const std::shared_ptr<drone_state> odom_data);
  virtual void resetGraph(const drone_state &state);
  virtual void register_camera(const std::string name,
                               const std::shared_ptr<transform> transform,
                               const std::shared_ptr<camera_info> camera_info);
  virtual void aruco_callback(const std::shared_ptr<std::vector<alphapilot::ArucoDetection>> msg, const std::string camera_name);
  virtual double get_optimization_time();

  virtual std::map<std::string, std::array<double, 3>> get_landmark_positions();

  virtual void run_optimize();

  std::map<int, std::list<alphapilot::Landmark>> group_gates();
  bool object_in_gate(std::list<alphapilot::Landmark> gate, std::string l_type);
  bool object_close_to_gate(std::list<alphapilot::Landmark> gate, Landmark l);

  virtual alphapilot::drone_state latest_state(bool optimize=false);

 private:
  virtual void add_pose_factor();
  virtual void add_priors(const drone_state &initial_state);
  virtual void add_factors();

  // ========== GENERIC VARS =======
  bool debug_ = true;
  // if any thing has updated the estimate of position
  bool position_update_ = false;
  // current estimate of the position of the drone
  alphapilot::drone_state current_pose_estimate_;

  // how long the most recent optimization took
  double optimization_time_ = 0.0;

  // flags to determine if factors are used
  bool use_pose_factors_ = true;
  bool use_range_factors_ = false;
  // even is disabled will create a state in the FG
  bool use_camera_factors_ = true;
  bool use_aruco_factors_ = true;

  // ========= GRAPH GENERICS ===========
  // current graph that gets updated and cleared after each optimization
  std::shared_ptr<gtsam::NonlinearFactorGraph> current_incremental_graph_;
  gtsam::ISAM2 isam_;

  // mutex locks
  std::mutex graph_lck_, pose_lck_, latest_state_lck_, optimization_lck_, landmark_lck_;

  // index of the current state
  int index_ = 0;

  // Current estimate of the state to be passed into factor graph
  gtsam::Pose3 current_position_guess_;
  gtsam::Vector3 current_velocity_guess_;

  // values to store above
  // X = pose, V = velocity, L = (x,y,z) of gate, A = offset to aruco
  std::shared_ptr<gtsam::Values> gtsam_current_state_initial_guess_;

  gtsam::ISAM2Params isam_parameters_;
  // entire history of the state, only enabled in debug mode
  gtsam::Values history_;

  // ========= PRIOR CONFIG =========
  prior_config prior_config_;

  // ========= POSE FACTOR HELPERS =========
  // number of pose messages
  int pose_message_count_ = 0;

  // current diff since last optimization for Pose3 between factor
  gtsam::Rot3 pose_rot_accum_;
  gtsam::Point3 pose_trans_accum_;
  // current diff since last optimization for vel between factor
  gtsam::Vector3 vel_change_accum_;
  // used to calculate the diff between current and last pose
  drone_state last_pose_state_;
  // noise model for pose and vel
  gtsam::noiseModel::Diagonal::shared_ptr odometry_pose_noise_;
  gtsam::noiseModel::Diagonal::shared_ptr odometry_vel_noise_;

  // ========== PROJECTION FACTOR =============
  // keeps track of the projection factors for each landmark
  std::map<std::string, gtsam::noiseModel::Diagonal::shared_ptr> object_noises_;
  gtsam::noiseModel::Diagonal::shared_ptr default_camera_noise_;
  std::map<std::string, gtsam_camera> camera_map;
  std::map<std::string, bool> got_detections_from_;
  std::map<std::string, std::array<double, 3>> landmark_locations_;
  std::map<std::string, int> landmark_id_map_;
  int gate_landmark_index_ = 0;

  gtsam::noiseModel::Diagonal::shared_ptr aruco_pose_noise_;
  gtsam::noiseModel::Diagonal::shared_ptr aruco_pose_prior_noise_;
  std::map<std::string, bool> aruco_got_detections_from_;
  // list of all indexes of aruco we have seen
  std::unordered_set<int> aruco_indexes_;
};
} // estimator
} // StateEstimator
#endif // FactorGraphEstimator_H_
