//
// Created by jason on 6/1/19.
//

#include <StateEstimator/estimation/FactorGraph/FactorGraphEstimator.h>

using namespace alphapilot::estimator;
using namespace alphapilot;
using namespace gtsam;

int main() {
  std::shared_ptr<drone_state> init_state = std::make_shared<drone_state>();
  init_state->z = 4.0;
  init_state->qx = 0.0;
  init_state->qy = 0.0;
  init_state->qz = 0.7071068;
  init_state->qw = 0.7071068;
  FactorGraphEstimator estimator(init_state, true);
  std::cout << "\ninit ended\n" << std::endl;

  std::cout << "\nstarting odom callback\n" << std::endl;
  std::shared_ptr<drone_state> reading_odom = std::make_shared<drone_state>();
  reading_odom->z = 2.0;
  reading_odom->x = 2.0;
  reading_odom->qx = 0.0;
  reading_odom->qy = 0.0;
  reading_odom->qz = 0.70710680;
  reading_odom->qw = 0.7071068;
  estimator.callback_odometry(reading_odom);
  std::cout << "\nodom callback ended\n" << std::endl;


  std::cout << "\nstarting imu callback\n" << std::endl;
  std::shared_ptr<IMU_readings> reading_imu = std::make_shared<IMU_readings>();
  reading_imu->dt = 0.1;
  reading_imu->x_accel = 1.0;
  reading_imu->z_accel = -9.81;
  //estimator.callback_imu(reading_imu);
  std::cout << "\nending imu callback\n" << std::endl;


  std::cout << "\nstarting camera callback\n" << std::endl;
  std::shared_ptr<std::map<std::string, std::pair<double, double>>> camera_reading = std::make_shared<std::map<std::string, std::pair<double, double>>>();
  //camera_reading->insert(std::make_pair("Gate10-1", std::make_pair(402.281, 195.785)));
  //camera_reading->insert(std::make_pair("Gate12-3", std::make_pair(55.6591, 147.801)));
  estimator.callback_cm(camera_reading);

  drone_state result = estimator.latest_state();
  estimator.resetGraph(init_state);

  std::cout << "\nstarting odom callback\n" << std::endl;
  reading_odom->x = 0;
  estimator.callback_odometry(reading_odom);
  std::cout << "\nodom callback ended\n" << std::endl;

  std::cout << "\nstarting camera callback\n" << std::endl;
  estimator.callback_cm(camera_reading);

  result = estimator.latest_state();
  std::cout << "\n\nfinished code\n\n" << std::endl;
  /*
  NonlinearFactorGraph current_incremental_graph_;
  // Values initialStateEstimate;
  ISAM2 isam;

  ISAM2Params parameters;
  parameters.relinearizeThreshold = 0.01;
  parameters.relinearizeSkip = 1;
  isam = ISAM2(parameters);
  int index_ = 0;

  // pose factor noise
  gtsam::noiseModel::Diagonal::shared_ptr odometry_pose_noise_;
  gtsam::noiseModel::Diagonal::shared_ptr odometry_vel_noise_;
  odometry_vel_noise_ = noiseModel::Diagonal::Sigmas(Vector3(0.1, 0.1, 0.1));
  Vector6 pose_noise;
  pose_noise << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
  odometry_pose_noise_ = noiseModel::Diagonal::Sigmas(Vector6(pose_noise));

  Pose3 pose_change_accum_ = Pose3(Rot3(), Point3(0.5,0,0));
  Vector3 vel_change_accum_;
  vel_change_accum_ << 1.0, 0.0, 0.0;
  gtsam::Pose3 current_position_guess_(Rot3(), Point3(0,0,1));
  gtsam::Vector3 current_velocity_guess_;
  gtsam::Values gtsam_current_state_initial_guess_;

  //priors
  // insert initial guesses of state
  {
    noiseModel::Diagonal::shared_ptr pose_noise_model = noiseModel::Diagonal::Sigmas((Vector(6)
        << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1).finished()); // rad,rad,rad,m, m, m
    noiseModel::Diagonal::shared_ptr velocity_noise_model = noiseModel::Isotropic::Sigma(3, 0.1); // m/s
    noiseModel::Diagonal::shared_ptr bias_noise_model = noiseModel::Isotropic::Sigma(6, 1e-3);
    gtsam_current_state_initial_guess_.insert(symbol_shorthand::X(index_),
                                              current_position_guess_);
    gtsam_current_state_initial_guess_.insert(symbol_shorthand::V(index_),
                                              current_velocity_guess_);
    current_incremental_graph_.add(PriorFactor<Pose3>(symbol_shorthand::X(index_),
                                                      current_position_guess_,
                                                      pose_noise_model));
    current_incremental_graph_.add(PriorFactor<Vector3>(symbol_shorthand::V(index_),
                                                        current_velocity_guess_,
                                                        velocity_noise_model));
  }

  index_++;
  //add factors
  // add constraint on the poses

  current_incremental_graph_.emplace_shared<BetweenFactor<Pose3>>(symbol_shorthand::X(index_-1),
      symbol_shorthand::X(index_), pose_change_accum_, odometry_pose_noise_);
  // add constraint on the velocity
  current_incremental_graph_.emplace_shared<BetweenFactor<Vector3>>(symbol_shorthand::V(index_-1),
      symbol_shorthand::V(index_), vel_change_accum_, odometry_vel_noise_);

  gtsam_current_state_initial_guess_.insert(symbol_shorthand::X(index_),
                                            current_position_guess_ * pose_change_accum_);
  Vector3 temp_vel = current_velocity_guess_ + vel_change_accum_;
  gtsam_current_state_initial_guess_.insert(symbol_shorthand::V(index_), temp_vel);

  std::cout << "\n\n incremental graph" << std::endl;
  current_incremental_graph_.print();
  std::cout << "\n\n guess state guesses" << std::endl;
  gtsam_current_state_initial_guess_.print();

  isam.update(current_incremental_graph_, gtsam_current_state_initial_guess_);
  */
  return 0;

}
