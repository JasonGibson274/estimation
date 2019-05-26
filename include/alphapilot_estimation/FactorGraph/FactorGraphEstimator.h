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

#include <alphapilot/Utils.h>

#include <map>
#include <mutex>
#include <memory>

namespace alphapilot {
namespace estimator {
class FactorGraphEstimator {
public:
	FactorGraphEstimator();
	~FactorGraphEstimator() = default;

  virtual void callback_cm(std::map<int l_id, std::pair<double, double>>
  												 landmark_data);
  // TODO: Figure out what data structure is used for range finders
  virtual void callback_range(int rangestuff);
  virtual void callback_imu(IMU_readings imu_data);
  /** Takes the latest state estimation from gtsam and returns it as a
   * drone state for other code to use
   * @return
   */
  virtual drone_state lastest_state();
private:
	// Declare variables here and also add_imu
	virtual void run_optimimize();
	virtual void add_imu_factor();
	gtsam::InitialEstimate gtsam_current_state_initial_guess_;

	// Updated in callback_imu
	std::shared_ptr<gtsam::Pose3> current_position_guess_;
	std::shared_ptr<gtsam::Vector3> current_velocity_guess_;
	std::shared_ptr<gtsam::IMUBias> current_bias_guess_;
	std::shared_ptr<gtsam::Key> bias_index_;

	std::mutex graph_lck_, preintegrator_lck_;
	std::shared_ptr<gtsam::NonLinearFactorGraph> current_incremental_graph_;
	std::shared_ptr<std::map<int, gtsam::SmartProjectionPoseFactor<gtsam::Cal3_S2>>> landmark_factors_;
	std::shared_ptr<gtsam::ISAM2> isam_;
	std::shared_ptr<gtsam::PreintegratedImuMeasurements> preintegrator_imu_;
	gtsam::Cal3_S2::shared_ptr K_;
	int index = 0;
	// Add the NoiseModels for IMU and Camera and RangeFinder
	gtsam::noiseModel::Isotropic cam_measurement_noise_;
	gtsam::noiseModel::Diagonal bias_noise_;
}
} // alphapilot
} // estimator
#endif // FactorGraphEstimator_H_