/**
Description goes here and also licesnse?
Authors: Bogdan Vlahov and Jason Gibson
**/
#ifndef FactorGraphEstimator_H_
#define FactorGraphEstimator_H_

#include <gtsam/geometry/SimpleCamera.h>
#include <gtsam/geometry/Quaternion.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/sam/RangeFactor.h>

#include <alphapilot_common/Utils.h>

#include <map>
#include <mutex>
#include <memory>
#include <iostream>
#include <cstdio>
#include <gtsam/sam/RangeFactor.h>

#include <yaml-cpp/yaml.h>
#include <unordered_set>
#include <tuple>
#include <queue>

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

struct imu_factor_params {
  // factor specific
  double accelNoiseSigma = 2.0;
  double gyroNoiseSigma = 0.1;
  double accelBiasRwSigma = 0.1;
  double gyroBiasRwSigma = 0.1;
  double integrationErrorCov = 1e-4;
  double biasAccOmegaInt = 0.1;

  // general
  bool useImuFactor = true;
  int imuBiasIncr = 5;
  bool invertX = false;
  bool invertY = false;
  bool invertZ = false;

  // bias noise
  std::vector<double> biasNoise = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
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
  double initial_bias_noise = 0.1;
};

struct estimator_config {
  bool debug = false;
  // initial time to start the factor graph at
  double time = 0.0;
  isam_parameters isamParameters;
  imu_factor_params imuFactorParams;
  pose_factor_params poseFactorParams;
  camera_factor_params cameraFactorParams;
  prior_config priorConfig;
};

struct detection {
  std::string id = "";
  gtsam::Point2 point = gtsam::Point2(-1,-1);
  int index = 0;
};

struct optimization_stats {
  double optimizationTime = 0.0;
  double optimizationTimeAvg = 0.0;
  int optimizationIterations = 0;
  int variablesReeliminated = 0;
  int variablesRelinearized = 0;
  double errorBefore = 0.0;
  double errorAfter = 0.0;
};

std::ostream& operator <<(std::ostream& os, const optimization_stats& stats) {
  os << "\nOptimization Results:\n"
         << "\nReelimintated: " << stats.variablesReeliminated
         << "\nRelinearized: " << stats.variablesRelinearized;
  if(stats.errorBefore) {
    os << "\nError Before: " << stats.errorBefore;
  }
  if(stats.errorAfter) {
   os << "\nError After : " << stats.errorAfter;
  }
  os << "\noptimizationIterations: " << stats.optimizationIterations
     << "\noptimizationTime: " << stats.optimizationTime
     << "\noptimizationTimeAvg: " << stats.optimizationTimeAvg;
  return os;
};

class FactorGraphEstimator {
 public:
  FactorGraphEstimator(const std::string &config_file, const std::string& full_path);

  virtual void callback_cm(std::shared_ptr<alphapilot::GateDetections> detection_data);
  // TODO: Figure out what data structure is used for range finders
  virtual void callback_range(const int rangestuff);
  virtual void callback_odometry(const std::shared_ptr<drone_state> odom_data);
  virtual void callback_imu(const std::shared_ptr<IMU_readings> imu_data);
  virtual void aruco_callback(const std::shared_ptr<alphapilot::ArucoDetections> msg);
  virtual void timing_callback(const double timestamp);
  virtual optimization_stats get_optimization_stats();

  virtual std::vector<alphapilot::Landmark> get_landmark_positions();

  virtual void run_optimize();

  void group_gates();
  bool object_in_gate(std::list<alphapilot::Landmark> gate, std::string l_type);
  bool object_close_to_gate(std::list<alphapilot::Landmark> gate, Landmark l);

  virtual alphapilot::drone_state latest_state(bool optimize=false);

  virtual void add_projection_prior(std::shared_ptr<Landmark> msg);

  virtual std::vector<alphapilot::Gate> get_gates();

  std::vector<alphapilot::PointWithCovariance> get_aruco_locations();

 private:
  virtual void register_camera(const std::string name,
                               const std::shared_ptr<gtsam::Point3> translation,
                               const std::shared_ptr<gtsam::Rot3> rotation,
                               const std::shared_ptr<camera_info> camera_info);
  virtual void add_pose_factor();
  virtual void add_priors(const drone_state &initial_state);
  virtual void add_imu_factor();
  void propagate_imu(gtsam::Vector3 acc, gtsam::Vector3 angular_vel, double dt);
  int find_camera_index(double time);
  void print_projection(int image_index, gtsam::Point3 position, gtsam_camera camera, gtsam::Point2 detections_coords);
  void calculate_gate_centers();
  void assign_gate_ids(std::shared_ptr<GateDetections> detection_msg);
  void print_values(std::shared_ptr<gtsam::Values> values);

  // ========== GENERIC VARS =======
  bool debug_ = true;
  // if any thing has updated the estimate of position
  bool position_update_ = false;
  // current estimate of the position of the drone
  alphapilot::drone_state current_pose_estimate_;

  // how long the most recent optimization took
  optimization_stats optimization_stats_;

  // flags to determine if factors are used
  bool use_pose_factors_ = true;
  bool use_range_factors_ = false;
  bool use_imu_factors_ = true;
  bool use_camera_factors_ = true;
  // even is disabled will create a state in the FG
  bool use_aruco_factors_ = true;

  // ========= GRAPH GENERICS ===========
  // current graph that gets updated and cleared after each optimization
  std::shared_ptr<gtsam::NonlinearFactorGraph> current_incremental_graph_;
  gtsam::ISAM2 isam_;
  std::map<int, double> time_map_;

  // mutex locks
  std::mutex graph_lck_; // controls isam_ and current_incremental_graph_
  std::mutex pose_lck_; // controls temp accum variables for pose factor, *accum_
  std::mutex latest_state_lck_; // controls the current vel and pos guess as well as cur state
  std::mutex optimization_lck_; // lock to prevent multiple optimization running concurrently
  std::mutex landmark_lck_; // lock to prevent reading and writing to landmark_locations_
  std::mutex preintegrator_lck_; // lock to control preintegrator
  std::mutex gate_lck_; // lock to control gate_centers_
  std::mutex aruco_locations_lck_; // lock to control aruco positions

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

  // ========== IMU ===========================
  gtsam::PreintegratedImuMeasurements preintegrator_imu_;
  // the number of IMU messages currently integrated
  double last_imu_time_ = -1;
  int bias_index_ = 0;
  int imu_bias_incr_ = 1;
  bool use_imu_prop_ = true;
  bool invert_x_ = false;
  bool invert_y_ = false;
  bool invert_z_ = false;
  bool use_imu_bias_ = true;
  const double GRAVITY = 9.81;
  gtsam::noiseModel::Diagonal::shared_ptr bias_noise_;
  gtsam::imuBias::ConstantBias current_bias_guess_;

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
  std::map<int, alphapilot::Landmark> landmark_locations_;
  std::vector<alphapilot::PointWithCovariance> aruco_locations_;
  std::vector<alphapilot::Gate> gate_centers_;
  // TODO change to pair of id and type
  std::map<int, std::string> id_to_landmark_map_;
  std::map<std::string, int> landmark_to_id_map_;
  int gate_landmark_index_ = 0;
  // how close a timestamp has to be to the state time
  double pairing_threshold_ = 0.1;
  gtsam::noiseModel::Diagonal::shared_ptr landmark_prior_noise_;
  bool reassign_gate_ids_ = true;

  // ========== ARUCO FACTOR =============
  gtsam::noiseModel::Diagonal::shared_ptr aruco_camera_noise_;
  gtsam::noiseModel::Diagonal::shared_ptr aruco_pose_prior_noise_;
  std::map<std::string, bool> aruco_got_detections_from_;
  // list of all indexes of aruco we have seen
  std::unordered_set<int> aruco_indexes_;
};
} // estimator
} // StateEstimator
#endif // FactorGraphEstimator_H_
