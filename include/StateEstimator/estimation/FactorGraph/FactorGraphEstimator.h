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

#include <alphapilot_common/Utils.h>
#include <alphapilot_common/GateConstraints.h>

#include <map>
#include <mutex>
#include <memory>
#include <iostream>
#include <cstdio>
#include <gtsam/sam/RangeFactor.h>

#include <yaml-cpp/yaml.h>

// TODO make callbacks pass by const reference to shared pointer
namespace alphapilot {
namespace estimator {
struct gtsam_camera {
  gtsam::Pose3 transform;
  boost::shared_ptr<gtsam::Cal3_S2> K;
};

struct prior_config {
  drone_state initial_state;
  double initial_vel_noise = 0.1;
  std::vector<double> initial_pose_noise = {0.05, 0.05, 0.05, 0.25, 0.25, 0.25};//rad, rad, rad, m,m,m
  double initial_bias_noise = 0.1;
};

struct optimization_stats {
  double optimizationTime = 0.0;
  double optimizationTimeAvg = 0.0;
  double getLandmarksTime = 0.0;
  double getLandmarksTimeAvg = 0.0;
  int optimizationIterations = 0;
  int variablesReeliminated = 0;
  int variablesRelinearized = 0;
  double errorBefore = 0.0;
  double errorAfter = 0.0;
};

struct detection_header {
  std::string type = "";
  std::string id = "";
  std::string camera = "";
};

bool operator <(const detection_header& x, const detection_header& y) {
  return x.camera+"_"+x.type+"_"+x.id <  y.camera+"_"+y.type+"_"+y.id;
}

struct smart_detection {
  gtsam::Point2 detection;
  int state_index;
  detection_header header;
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
     << "\noptimizationTimeAvg: " << stats.optimizationTimeAvg
     << "\ngetLandmarksTime: " << stats.getLandmarksTime
     << "\ngetLandmarksTimeAvg: " << stats.getLandmarksTimeAvg;
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
  virtual void smart_projection_callback(const std::shared_ptr<alphapilot::CameraDetections> detections);
  virtual optimization_stats get_optimization_stats();
  virtual void add_constraints_to_gates(std::map<std::string, double> size_map);

  virtual std::vector<alphapilot::Landmark> get_landmark_positions();

  /*
   * returns false if another optimization is running and it fails to lock, or if no position updates
   */
  virtual bool run_optimize();

  virtual alphapilot::drone_state latest_state(bool optimize=false);

  virtual void add_projection_prior(std::shared_ptr<Landmark> msg);

  virtual std::vector<alphapilot::Gate> get_gates();

  std::vector<alphapilot::PointWithCovariance> get_aruco_locations();

  std::map<std::string, std::vector<alphapilot::PointWithCovariance>> get_smart_locations();

  void register_camera(const std::string name,
                       const std::vector<double> translation,
                       const std::vector<double> rotation,
                       const std::vector<double> intrinsics);

  void calculate_gate_centers();

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
  bool assign_gate_ids(std::shared_ptr<GateDetections> detection_msg, int image_index);
  void print_values(std::shared_ptr<gtsam::Values> values);
  gtsam::Point3 generate_aruco_priors(const gtsam::Pose3& pos_copy, const alphapilot::Pose& pose, int index, double size);

  // ========== GENERIC VARS =======
  bool debug_ = true;
  bool full_history_debug_ = false; // TODO config
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
  std::mutex gate_lck_; // lock to control gate_centers_
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
   * G(t) center of gate with id t
   */
  std::shared_ptr<gtsam::Values> current_state_guess_;

  // entire history of the state, only enabled in debug mode
  gtsam::Values history_;

  // ========== IMU ===========================
  gtsam::PreintegratedImuMeasurements preintegrator_imu_;
  bool use_imu_prop_ = true;
  bool imu_debug_ = false;
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
  gtsam::noiseModel::Diagonal::shared_ptr projection_landmark_noise_;
  gtsam::noiseModel::Constrained::shared_ptr projection_constraint_noise_;
  std::unordered_set<std::string> gate_center_inited_;
  double projection_constraint_noise_val_;
  // will estimate gate center in graph
  bool use_gate_center_ = false;
  gtsam::noiseModel::Diagonal::shared_ptr gate_range_noise_;
  // will turn off projection constraints
  bool use_projection_constraints_ = false;
  // camera name to camera parameters
  std::map<std::string, gtsam_camera> camera_map;
  // L index, and actual landmark
  std::map<int, alphapilot::Landmark> landmark_locations_;
  std::vector<alphapilot::Gate> gate_centers_;
  // TODO change to pair of id and type
  std::map<detection_header, int> landmark_to_id_map_;
  int gate_landmark_index_ = 0;
  gtsam::noiseModel::Diagonal::shared_ptr landmark_prior_noise_;
  // will reassign gate ids based on proximity to landmarks
  bool reassign_gate_ids_ = true;
  // maximum distance between estimated gate position to be classified correctly
  double gate_dist_threshold_ = 10;

  // ========== ARUCO FACTOR =============
  gtsam::noiseModel::Diagonal::shared_ptr aruco_camera_noise_;
  gtsam::noiseModel::Diagonal::shared_ptr aruco_pose_prior_noise_;
  gtsam::noiseModel::Diagonal::shared_ptr aruco_range_noise_;
  gtsam::noiseModel::Constrained::shared_ptr aruco_constraint_noise_;
  std::vector<alphapilot::PointWithCovariance> aruco_locations_;
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
  std::map<detection_header, gtsam::SmartProjectionPoseFactor<gtsam::Cal3_S2>::shared_ptr> id_to_smart_landmarks_;
  std::list<smart_detection> smart_detections_queue_;
  gtsam::SmartProjectionParams projection_params_;
  // type -> resulting point
  std::map<std::string, std::vector<alphapilot::PointWithCovariance>> smart_locations_;
  gtsam::noiseModel::Diagonal::shared_ptr smart_default_noise_;
  std::map<std::string, gtsam::noiseModel::Diagonal::shared_ptr> smart_object_noises_;
};
} // estimator
} // StateEstimator
#endif // FactorGraphEstimator_H_
