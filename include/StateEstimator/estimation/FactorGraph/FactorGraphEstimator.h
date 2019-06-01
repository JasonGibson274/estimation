/**
Description goes here and also licesnse?
Authors: Bogdan Vlahov and Jason Gibson
**/
#ifndef FactorGraphEstimator_H_
#define FactorGraphEstimator_H_

#include <gtsam/geometry/SimpleCamera.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/ImuFactor.h>
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
	explicit FactorGraphEstimator(std::shared_ptr<drone_state> initial_state);

  virtual void callback_cm(std::shared_ptr<std::map<std::string, std::pair<double, double>>> landmark_data);
  // TODO: Figure out what data structure is used for range finders
  virtual void callback_range(int rangestuff);
  virtual void callback_imu(std::shared_ptr<IMU_readings> imu_data);

  virtual void resetGraph(std::shared_ptr<drone_state> state);

private:
	// Declare variables here and also add_imu
	virtual void run_optimize();
	virtual void add_imu_factor();
  void propagate_imu(drone_state current_state, gtsam::Vector3 acc, gtsam::Vector3 angular_vel, double dt);
	std::shared_ptr<gtsam::Values> gtsam_current_state_initial_guess_;

	// Updated in callback_imu
	std::shared_ptr<gtsam::Pose3> current_position_guess_;
	std::shared_ptr<gtsam::Vector3> current_velocity_guess_;
	std::shared_ptr<gtsam::imuBias::ConstantBias> current_bias_guess_;
	gtsam::Key bias_index_{};

	std::mutex graph_lck_, preintegrator_lck_;
	std::shared_ptr<gtsam::NonlinearFactorGraph> current_incremental_graph_;
	std::map<std::string, gtsam::SmartProjectionPoseFactor<gtsam::Cal3_S2>*> landmark_factors_;
	std::shared_ptr<gtsam::ISAM2> isam_;
	gtsam::PreintegratedImuMeasurements preintegrator_imu_;
	gtsam::Cal3_S2 K_;
	int index = 0;
	// Add the NoiseModels for IMU and Camera and RangeFinder
	gtsam::noiseModel::Diagonal::shared_ptr cam_measurement_noise_;
	gtsam::noiseModel::Diagonal::shared_ptr bias_noise_;
};
} // estimator
} // StateEstimator
#endif // FactorGraphEstimator_H_