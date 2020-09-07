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
#include <gtsam/slam/SmartProjectionPoseFactor.h>

#include <utils/FileUtils.hpp>

#include <cstdio>
#include <iostream>
#include <map>
#include <mutex>
#include <memory>
#include <unordered_set>

// ROS variables only, can be replaced eventually but effort.
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/CameraInfo.h>

#include <autorally_estimation/ArucoDetections.h>
#include <autorally_estimation/CameraDetections.h>
#include <autorally_estimation/OptimizationStats.h>


// TODO make callbacks pass by const reference to shared pointer
namespace estimator {
struct GtsamCamera {
  gtsam::Pose3 transform;
  boost::shared_ptr<gtsam::Cal3_S2> k;
};

struct PriorConfig {
  gtsam::Pose3 state;
  gtsam::Vector3 velocity;
  double initial_vel_noise = 0.1;
  std::vector<double> initial_pose_noise = {0.05, 0.05, 0.05, 0.25, 0.25, 0.25};//rad, rad, rad, m,m,m
  double initial_bias_noise = 0.1;
};

struct OptimizationStats {
  double optimization_time = 0.0;
  double optimization_time_avg = 0.0;
  double get_landmarks_time = 0.0;
  double get_landmarks_time_avg = 0.0;
  int optimization_iterations = 0;
  int variables_reeliminated = 0;
  int variables_relinearized = 0;
  double error_before = 0.0;
  double error_after = 0.0;
};

struct DetectionHeader {
  std::string type = "";
  std::string id = "";
  std::string camera = "";
};

bool operator <(const DetectionHeader& x, const DetectionHeader& y) {
  return x.camera+"_"+x.type+"_"+x.id <  y.camera+"_"+y.type+"_"+y.id;
}

struct SmartDetection {
  gtsam::Point2 detection;
  int state_index;
  DetectionHeader header;
};

struct Landmark {
  std::string id = "";
  std::string type = "";
  geometry_msgs::Point point;
};

std::ostream& operator <<(std::ostream& os, const OptimizationStats& stats) {
  os << "\nOptimization Results:\n"
         << "\nReelimintated: " << stats.variables_reeliminated
         << "\nRelinearized: " << stats.variables_relinearized;
  if(stats.error_before) {
    os << "\nError Before: " << stats.error_before;
  }
  if(stats.error_after) {
   os << "\nError After : " << stats.error_after;
  }
  os << "\noptimizationIterations: " << stats.optimization_iterations
     << "\noptimizationTime: " << stats.optimization_time
     << "\noptimizationTimeAvg: " << stats.optimization_time_avg
     << "\ngetLandmarksTime: " << stats.get_landmarks_time
     << "\ngetLandmarksTimeAvg: " << stats.get_landmarks_time_avg;
  return os;
};

class FactorGraphEstimator {
 public:
  FactorGraphEstimator(const std::string &config_file);

  // TODO: Figure out what data structure is used for range finders
  virtual void CallbackOdometry(const std::shared_ptr<nav_msgs::Odometry> odom_data);
  virtual void CallbackImu(const std::shared_ptr<sensor_msgs::Imu> imu_data);
  virtual void ArucoCallback(const std::shared_ptr<autorally_estimation::ArucoDetections> msg);
  virtual void TimingCallback(const double timestamp);
  virtual void SmartProjectionCallback(const std::shared_ptr<autorally_estimation::CameraDetections> detections);
  virtual OptimizationStats GetOptimizationStats();

  virtual std::vector<Landmark> GetLandmarkPositions();

  /*
   * returns false if another optimization is running and it fails to lock, or if no position updates
   */
  virtual bool RunOptimize();

  virtual nav_msgs::Odometry LatestState(bool optimize=false);

  void AddProjectionPrior(std::shared_ptr<Landmark> msg);

  std::vector<geometry_msgs::PoseWithCovarianceStamped> GetArucoLocations();

  std::map<std::string, std::vector<geometry_msgs::PoseWithCovarianceStamped>> GetSmartLocations();

  geometry_msgs::PoseArray GetStateHistory();

  void AddArucoPrior(std::vector<double> position, int id);

  // general getters
  std::shared_ptr<gtsam::Values> getCurrentStateGuess() {return current_state_guess_;}
  std::shared_ptr<gtsam::Values> getHistory() {return history_;}
  std::shared_ptr<gtsam::NonlinearFactorGraph> getCurrentIncrementalGraph() {return current_incremental_graph_;}
  bool getDebugMode() {return debug_;}
  int getCurrentIndex() {return index_;}
  std::map<int, double> getTimeMap() {return time_map_;}
  bool getPositionUpdate() {return position_update_;}
  // prior getters
  PriorConfig getPriorConfig() {return prior_config_;}
  gtsam::ISAM2Params getISAMParams() {return isam_parameters_;}
  // camera getters
  double getPairingThreshold() {return pairing_threshold_;}
  std::map<std::string, GtsamCamera> getCameraMap() {return camera_map_;}
  // projection getters
  // aruco getters
  // imu getters
  bool getVerboseImu() {return imu_debug_;}
  bool getUseImuFactors() {return use_imu_factors_;}
  bool getUseImuBias() {return use_imu_bias_;}
  std::array<bool, 3> getInvertImuStatus() {
    return {invert_x_, invert_y_, invert_z_};
  }
  int getImuBiasIncr() {return imu_bias_incr_;}
  int getImuBiasIndex() {return bias_index_;}
  gtsam::noiseModel::Diagonal::shared_ptr getImuBiasNoise() {return bias_noise_;}
  gtsam::PreintegratedImuMeasurements getImuMeasurementsObject() {return preintegrator_imu_;}
  double getLastImuTime() {return last_imu_time_;}
  // pose factor
  bool getUsePoseFactor() {return use_pose_factors_;}
  gtsam::noiseModel::Diagonal::shared_ptr getOdometryPoseNoise() {return odometry_pose_noise_;}
  gtsam::noiseModel::Diagonal::shared_ptr getOdometryVelNoise() {return odometry_vel_noise_;}
  // smart projection factor params
  bool getUseSmartProjectionFactor() {return use_smart_pose_projection_factor_;}
  gtsam::SmartProjectionParams getSmartProjectionParams() {return projection_params_;}
  gtsam::noiseModel::Isotropic::shared_ptr getSmartProjectionNoise() {return smart_default_noise_;}


  virtual void PropagateImu(gtsam::Pose3& pose, gtsam::Vector3& vel, gtsam::PreintegratedImuMeasurements& preintegrator);
  // methods for adding specific factors
  virtual void AddPoseFactor();
  virtual void AddImuFactor();
  virtual void PrintValues(std::shared_ptr<gtsam::Values> values, std::string prefix);

  void RegisterCamera(const std::string name,
                               const std::shared_ptr<gtsam::Point3> translation,
                               const std::shared_ptr<gtsam::Rot3> rotation,
                               const std::shared_ptr<sensor_msgs::CameraInfo> camera_info);
  void AddPriors();
  int FindCameraIndex(double time);
  void PrintProjection(int image_index, gtsam::Point3 position, GtsamCamera camera, gtsam::Point2 detections_coords);

protected:
  // ========== GENERIC VARS =======
  bool debug_ = true;
  bool full_history_debug_ = false; // TODO config
  int history_limit = 100; // TODO implement
  // if any thing has updated the estimate of position
  bool position_update_ = false;
  // current estimate of the position of the drone
  nav_msgs::Odometry current_pose_estimate_;

  // how long the most recent optimization took
  OptimizationStats optimization_stats_;

  // flags to determine if factors are used
  bool use_pose_factors_ = true;
  bool use_range_factors_ = false;
  bool use_imu_factors_ = true;
  bool use_projection_factors_ = true;
  // even is disabled will create a state in the FG
  bool use_aruco_factors_ = true;

  // how close a timestamp has to be to the state time
  double pairing_threshold_ = 0.1;

  // ========= GRAPH GENERICS ===========
  // current graph that gets updated and cleared after each optimization
  std::shared_ptr<gtsam::NonlinearFactorGraph> current_incremental_graph_;

  gtsam::ISAM2 isam_;
  // isam parameters, set up in yaml config
  gtsam::ISAM2Params isam_parameters_;

  // maps an index to a time
  std::map<int, double> time_map_;

  // index of the current state
  int index_ = 0;

  // mutex locks
  std::mutex graph_lck_; // controls isam_ and current_incremental_graph_
  std::mutex pose_lck_; // controls temp accum variables for pose factor, *accum_
  std::mutex latest_state_lck_; // controls the current vel and pos guess as well as cur state
  std::mutex optimization_lck_; // lock to prevent multiple optimization running concurrently
  std::mutex landmark_lck_; // lock to prevent reading and writing to landmark_locations_
  std::mutex preintegrator_lck_; // lock to control preintegrator
  std::mutex aruco_locations_lck_; // lock to control aruco positions
  std::mutex smart_detections_lck_; // lock to control smart detections queue
  std::mutex smart_locations_lck_; // lock to control smart detection locations
  std::mutex timing_lck_; // lock to prevent duplicate timing calls

  // Current estimate of the state to be passed into factor graph
  gtsam::Pose3 current_position_guess_;
  gtsam::Vector3 current_velocity_guess_;

  // values to store above
  /*
   * X(t) pose at time t
   * V(t) velocity at time t
   * L(t) landmark with id t
   * A(201) top left corner of aruco marker 20
      * 1 == top left
      * 2 == top right
      * 3 == bottom right
      * 4 == bottom left
   */
  std::shared_ptr<gtsam::Values> current_state_guess_;

  // entire history of the state, only enabled in debug mode
  std::shared_ptr<gtsam::Values> history_;

  // ========== IMU ===========================
  std::vector<gtsam::ImuFactor> imu_queue_;
  gtsam::PreintegratedImuMeasurements preintegrator_imu_;
  bool use_imu_prop_ = true;
  bool imu_debug_ = false; // TODO rename
  double last_imu_time_ = -1;
  int bias_index_ = 0;
  int imu_bias_incr_ = 1;
  bool invert_x_ = false;
  bool invert_y_ = false;
  bool invert_z_ = false;
  bool use_imu_bias_ = true;
  const double GRAVITY = 9.81;
  gtsam::noiseModel::Diagonal::shared_ptr bias_noise_;
  gtsam::imuBias::ConstantBias current_bias_guess_;

  // ========= PRIOR CONFIG =========
  PriorConfig prior_config_;

  // ========= POSE FACTOR HELPERS =========
  // number of pose messages
  int pose_message_count_ = 0;

  // current diff since last optimization for Pose3 between factor
  gtsam::Rot3 pose_rot_accum_;
  gtsam::Point3 pose_trans_accum_;
  // current diff since last optimization for vel between factor
  gtsam::Vector3 vel_change_accum_;
  // used to calculate the diff between current and last pose
  nav_msgs::Odometry last_pose_state_;
  // noise model for pose and vel
  gtsam::noiseModel::Diagonal::shared_ptr odometry_pose_noise_;
  gtsam::noiseModel::Diagonal::shared_ptr odometry_vel_noise_;

  // ========== PROJECTION FACTOR =============
  // keeps track of the projection factors for each landmark
  std::map<std::string, gtsam::noiseModel::Diagonal::shared_ptr> object_noises_;
  gtsam::noiseModel::Diagonal::shared_ptr default_camera_noise_;
  gtsam::noiseModel::Diagonal::shared_ptr projection_landmark_noise_;
  gtsam::noiseModel::Constrained::shared_ptr projection_constraint_noise_;
  double projection_constraint_noise_val_;
  // will turn off projection constraints
  bool use_projection_constraints_ = false;
  // camera name to camera parameters
  std::map<std::string, GtsamCamera> camera_map_;
  // L index, and actual landmark
  std::map<int, Landmark> landmark_locations_;
  // TODO change to pair of id and type
  std::map<DetectionHeader, int> landmark_to_id_map_;
  int landmark_index_ = 0;
  gtsam::noiseModel::Diagonal::shared_ptr landmark_prior_noise_;

  // ========== ARUCO FACTOR =============
  gtsam::noiseModel::Diagonal::shared_ptr aruco_camera_noise_;
  gtsam::noiseModel::Diagonal::shared_ptr aruco_pose_prior_noise_;
  gtsam::noiseModel::Diagonal::shared_ptr aruco_range_noise_;
  gtsam::noiseModel::Constrained::shared_ptr aruco_constraint_noise_;
  std::vector<geometry_msgs::PoseWithCovarianceStamped> aruco_locations_;
  // list of all indexes of aruco we have seen
  std::unordered_set<int> aruco_indexes_;
  bool use_aruco_constraints_ = false;
  bool use_range_for_aruco_ = true;
  bool use_aruco_prior_ = true;
  bool use_projection_debug_ = false;
  // distance from corner to corner of aruco
  double aruco_length_ = 0.2;

  // ========== SMART POSE PROJECTION FACTOR =============
  bool use_smart_pose_projection_factor_ = false;
  // header (id and type) to factor
  std::map<DetectionHeader, gtsam::SmartProjectionPoseFactor<gtsam::Cal3_S2>::shared_ptr> id_to_smart_landmarks_;
  std::list<SmartDetection> smart_detections_queue_;
  gtsam::SmartProjectionParams projection_params_;
  // type -> resulting point
  std::map<std::string, std::vector<geometry_msgs::PoseWithCovarianceStamped>> smart_locations_;
  gtsam::noiseModel::Isotropic::shared_ptr smart_default_noise_;
  std::map<std::string, gtsam::noiseModel::Diagonal::shared_ptr> smart_object_noises_;
};
} // estimator
#endif // FactorGraphEstimator_H_
