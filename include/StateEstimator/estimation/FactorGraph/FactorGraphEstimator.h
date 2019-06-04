/**
Description goes here and also licesnse?
Authors: Bogdan Vlahov and Jason Gibson
**/
#ifndef FactorGraphEstimator_H_
#define FactorGraphEstimator_H_

#include <gtsam/geometry/SimpleCamera.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/SmartProjectionPoseFactor.h>

#include <StateEstimator/Utils.h>

#include <map>
#include <mutex>
#include <memory>

#include <StateEstimator/estimation/Estimator.h>


// TODO make callbacks pass by const reference to shared pointer
namespace alphapilot {
namespace estimator {
class FactorGraphEstimator : Estimator {
public:
  FactorGraphEstimator(const std::shared_ptr<drone_state>& initial_state, bool debug);

  virtual void callback_cm(std::shared_ptr<std::map<std::string, std::pair<double, double>>> landmark_data);
  // TODO: Figure out what data structure is used for range finders
  virtual void callback_range(int rangestuff);
  virtual void callback_imu(std::shared_ptr<IMU_readings> imu_data);
  virtual void callback_odometry(std::shared_ptr<drone_state> odom_data);

  virtual void resetGraph(std::shared_ptr<drone_state> state);

  virtual void run_optimize();

  drone_state latest_state() override;

private:
	virtual void add_imu_factor();
	virtual void add_pose_factor();
	virtual void add_priors(std::shared_ptr<drone_state> initial_state);
	virtual void add_factors();
    void propagate_imu(gtsam::Vector3 acc, gtsam::Vector3 angular_vel, double dt);

    // ========== GENERIC VARS =======
    bool debug_ = true;

    // ========= POSE FACTOR HELPERS =========
	// current diff since last optimization for Pose3 between factor
	gtsam::Pose3 pose_change_accum_;
	// current diff since last optimization for vel between factor
	gtsam::Vector3 vel_change_accum_;
	drone_state last_optimized_pose_;

	// ========= GRAPH GENERICS ===========
	// current graph that gets updated and cleared after each optimization
	gtsam::NonlinearFactorGraph current_incremental_graph_;
    gtsam::ISAM2 isam_;

    // mutex locks
	std::mutex graph_lck_, preintegrator_lck_;

	// index of the current state
    int index_ = 0;
    // index of IMU bias, increments differently based on imu_bias_incr_
	int bias_index_ = 0;
	int imu_bias_incr_ = 5;

	// Current estimate of the state to be passed into factor graph
	gtsam::Pose3 current_position_guess_;
	gtsam::Vector3 current_velocity_guess_;
	gtsam::imuBias::ConstantBias current_bias_guess_;

	// values to store above
    gtsam::Values gtsam_current_state_initial_guess_;

    // ========== PROJECTION FACTOR =============
	// keeps track of the projection factors for each landmark
	std::map<std::string, gtsam::SmartProjectionPoseFactor<gtsam::Cal3_S2>*> landmark_factors_;
    boost::shared_ptr<gtsam::Cal3_S2> K_;

    // ========== IMU ===========================
	gtsam::PreintegratedImuMeasurements preintegrator_imu_;
	// the number of IMU messages currently integrated
	int imu_meas_count_ = 0;

	// ============ NOISE ============
	// Add the NoiseModels for IMU and Camera and RangeFinder
	gtsam::noiseModel::Diagonal::shared_ptr cam_measurement_noise_;
	gtsam::noiseModel::Diagonal::shared_ptr bias_noise_;
	gtsam::noiseModel::Diagonal::shared_ptr odometry_pose_noise_;
    gtsam::noiseModel::Diagonal::shared_ptr odometry_vel_noise_;
};
} // estimator
} // StateEstimator
#endif // FactorGraphEstimator_H_