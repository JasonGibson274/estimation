/**
Description goes here and also licesnse?
Authors: Bogdan Vlahov and Jason Gibson
**/
#ifndef FactorGraphEstimator_H_
#define FactorGraphEstimator_H_
#include <map>
#include <gtsamstuff>
#include <mutex>
#include <memory>
#include <Utils.h>

namespace alphapilot_estimator {
class FactorGraphEstimator {
public:
	FactorGraphEstimator();
	~FactorGraphEstimator() = default;

  // Define methods here
  virtual void callback_cm(imagestuff);
  virtual void callback_range(rangestuff); // TODO: Implement this shit
  virtual void callback_imu(imustuff);
private:
	// Declare variables here and also add_imu
	virtual void run_optimimize();
	virtual void add_imu_factor();
	gtsam::InitialEstimate gtsam_current_state_initial_guess_;

	// Updated in callback_imu
	std::shared_ptr<gtsam::Pose3> current_position_guess_;
	std::shared_ptr<gtsam::Vector3> current_velocity_guess_;
	std::shared_ptr<gtsam::IMUBias> current_bias_guess_;

	std::mutex graph_lck_, preintegrator_lck_;
	std::shared_ptr<gtsam::NonLinearFactorGraph> current_incremental_graph_;
	std::shared_ptr<std::map<int, gtsam::SmartProjectionPoseFactor<gtsam::Cal3_S2>>> landmark_factors_;
	std::shared_ptr<gtsam::ISAM2> isam_;
	std::shared_ptr<gtsam::PreintegratedImuMeasurements> preintegrator_imu_;
	int index;
	// Add the NoiseModels for IMU and Camera and RangeFinder


}
} // alphapilot_estimator
#endif // FactorGraphEstimator_H_