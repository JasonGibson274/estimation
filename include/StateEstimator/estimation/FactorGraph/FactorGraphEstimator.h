/**
Description goes here and also licesnse?
Authors: Bogdan Vlahov and Jason Gibson
**/
#ifndef FactorGraphEstimator_H_
#define FactorGraphEstimator_H_

#include <gtsam/geometry/SimpleCamera.h>
#include <gtsam/geometry/Quaternion.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/SmartProjectionPoseFactor.h>
#include <gtsam/slam/SmartProjectionPoseFactor.h>

#include <alphapilot_common/Utils.h>

#include <map>
#include <mutex>
#include <memory>
#include <iostream>
#include <cstdio>

#include <StateEstimator/estimation/Estimator.h>

#include <yaml-cpp/yaml.h>

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

class FactorGraphEstimator : Estimator {
 public:
  explicit FactorGraphEstimator(const estimator_config &estimator_config);

#if ENABLE_YAML
  FactorGraphEstimator(const std::string &config_file, const std::string& full_path);
#endif

  virtual void callback_cm(std::shared_ptr<std::map<std::string, std::pair<double, double>>> landmark_data,
                           std::string camera_name);
  // TODO: Figure out what data structure is used for range finders
  virtual void callback_range(const int rangestuff);
  virtual void callback_imu(const std::shared_ptr<IMU_readings> imu_data);
  virtual void callback_odometry(const std::shared_ptr<drone_state> odom_data);
  virtual void resetGraph(const drone_state &state);
  virtual void register_camera(const std::string name,
                               const std::shared_ptr<transform> transform,
                               const std::shared_ptr<camera_info> camera_info);
  virtual double get_optimization_time();

  virtual std::map<std::string, std::array<double, 3>> get_landmark_positions();

  virtual void run_optimize();

  double optimize_hz();

  drone_state latest_state(bool optimize=false) override;

  std::map<int, std::list<alphapilot::Landmark>> group_gates();
  bool object_in_gate(std::list<alphapilot::Landmark> gate, std::string l_type);
  bool object_close_to_gate(std::list<alphapilot::Landmark> gate, Landmark l);

 private:
  virtual void add_imu_factor();
  virtual void add_pose_factor();
  virtual void add_priors(const drone_state &initial_state);
  virtual void add_factors();
  void propagate_imu(gtsam::Vector3 acc, gtsam::Vector3 angular_vel, double dt);

  // ========== GENERIC VARS =======
  bool debug_ = true;
  // if any thing has updated the estimate of position
  bool position_update_ = false;
  double optimize_hz_ = 10;

  // how long the most recent optimization took
  double optimization_time_ = 0.0;

  // flags to determine if factors are used
  bool use_pose_factors_ = true;
  bool use_imu_factors_ = false;
  bool use_range_factors_ = false;
  // work differently TODO document
  bool use_camera_factors_ = true;

  int previous_optimization_index_ = 0;

  prior_config prior_config_;

  gtsam::ISAM2Params isam_parameters_;

  gtsam::Values history_;

  // ========= POSE FACTOR HELPERS =========
  // number of pose messages
  int pose_message_count_ = 0;
  // current diff since last optimization for Pose3 between factor
  gtsam::Rot3 pose_rot_accum_;
  gtsam::Point3 pose_trans_accum_;
  // current diff since last optimization for vel between factor
  gtsam::Vector3 vel_change_accum_;
  drone_state last_pose_state_;

  // ========= GRAPH GENERICS ===========
  // current graph that gets updated and cleared after each optimization
  std::shared_ptr<gtsam::NonlinearFactorGraph> current_incremental_graph_;
  gtsam::ISAM2 isam_;

  // mutex locks
  std::mutex graph_lck_, preintegrator_lck_, pose_lck_, latest_state_lck_, optimization_lck_;

  // index of the current state
  int index_ = 0;
  // index of IMU bias, increments differently based on imu_bias_incr_
  int bias_index_ = 0;
  int imu_bias_incr_ = 1;

  // Current estimate of the state to be passed into factor graph
  gtsam::Pose3 current_position_guess_;
  gtsam::Vector3 current_velocity_guess_;
  gtsam::imuBias::ConstantBias current_bias_guess_;

  // values to store above
  std::shared_ptr<gtsam::Values> gtsam_current_state_initial_guess_;

  // ========== PROJECTION FACTOR =============
  // keeps track of the projection factors for each landmark
  std::map<std::string, gtsam::SmartProjectionPoseFactor<gtsam::Cal3_S2>::shared_ptr> landmark_factors_;
  std::map<std::string, gtsam::noiseModel::Diagonal::shared_ptr> object_noises_;
  gtsam::noiseModel::Diagonal::shared_ptr default_camera_noise_;
  std::map<std::string, gtsam_camera> camera_map;
  std::list<detection> detections_;
  gtsam::SmartProjectionParams projection_params_;
  std::map<std::string, bool> got_detections_from_;

  // ========== IMU ===========================
  gtsam::PreintegratedImuMeasurements preintegrator_imu_;
  // the number of IMU messages currently integrated
  double last_imu_time_ = -1;
  int imu_meas_count_ = 0;
  bool invert_x_ = false;
  bool invert_y_ = false;
  bool invert_z_ = false;
  bool use_imu_bias_ = true;
  const double GRAVITY = 9.81;

  // ============ NOISE ============
  // Add the NoiseModels for IMU and Camera and RangeFinder
  gtsam::noiseModel::Diagonal::shared_ptr bias_noise_;
  gtsam::noiseModel::Diagonal::shared_ptr odometry_pose_noise_;
  gtsam::noiseModel::Diagonal::shared_ptr odometry_vel_noise_;
};
} // estimator
} // StateEstimator
#endif // FactorGraphEstimator_H_
