//
// Created by jason on 6/1/19.
//

#include <StateEstimator/estimation/FactorGraph/FactorGraphEstimator.h>

using namespace alphapilot::estimator;
using namespace alphapilot;
using namespace gtsam;

int main() {
  estimator_config config;
  config.time = 0.0;
  config.priorConfig.initial_state.z = 1.0;
  config.priorConfig.initial_state.qx = 0.0;
  config.priorConfig.initial_state.qy = 0.0;
  config.priorConfig.initial_state.qz = 0.0;
  config.priorConfig.initial_state.qw = 1.0;
  config.debug = true;
  config.imuFactorParams.useImuFactor = true;
  config.cameraFactorParams.useCameraFactor = true;
  config.poseFactorParams.usePoseFactor = false;
  FactorGraphEstimator estimator(config);
  //estimator.resetGraph(config.priorConfig.initial_state);
  std::cout << "\ninit ended\n" << std::endl;

  std::shared_ptr<camera_info> K = std::make_shared<camera_info>();
  K->fx = 548.4088134765625;
  K->fy = 548.4088134765625;
  K->s = 0.0;
  K->u0 = 512.0;
  K->v0 = 384.0;
  std::shared_ptr<transform> transform = std::make_shared<alphapilot::transform>();
  transform->qw = -0.5;
  transform->qx = 0.5;
  transform->qy = -0.5;
  transform->qz = 0.5;
  estimator.register_camera("Camera 1", transform, K);

  std::cout << "\nstarting odom callback\n" << std::endl;
  std::shared_ptr<drone_state> reading_odom = std::make_shared<drone_state>();
  reading_odom->z = 1.0;
  reading_odom->y = 0.05;
  reading_odom->x = 0.05;
  reading_odom->x_dot = 0.1;
  reading_odom->y_dot = 0.1;
  reading_odom->qx = 0.0;
  reading_odom->qy = 0.0;
  reading_odom->qz = 0.0;
  reading_odom->qw = 1.0;
  estimator.callback_odometry(reading_odom);
  std::cout << "\nodom callback ended\n" << std::endl;


  std::cout << "\nstarting imu callback\n" << std::endl;
  std::shared_ptr<IMU_readings> reading_imu = std::make_shared<IMU_readings>();
  reading_imu->time = 0.1;
  reading_imu->x_accel = 1.0;
  reading_imu->y_accel = 1.0;
  reading_imu->z_accel = 9.81;
  reading_imu->roll_vel = 0.0;
  reading_imu->pitch_vel = 0.0;
  reading_imu->yaw_vel = 0.0;
  estimator.callback_imu(reading_imu);

  reading_imu->time = 0.2;
  estimator.callback_imu(reading_imu);

  reading_imu->time = 0.3;
  estimator.callback_imu(reading_imu);

  reading_imu->time = 0.4;
  estimator.callback_imu(reading_imu);

  reading_imu->time = 0.5;
  estimator.callback_imu(reading_imu);

  reading_imu->time = 0.6;
  estimator.callback_imu(reading_imu);

  reading_imu->time = 0.7;
  estimator.callback_imu(reading_imu);

  reading_imu->time = 0.8;
  estimator.callback_imu(reading_imu);

  reading_imu->time = 0.9;
  estimator.callback_imu(reading_imu);

  reading_imu->time = 1.0;
  estimator.callback_imu(reading_imu);
  std::cout << "\nending imu callback\n" << std::endl;


  std::cout << "\nstarting camera callback\n" << std::endl;
  std::shared_ptr<std::map<std::string, std::pair<double, double>>> camera_reading = std::make_shared<std::map<std::string, std::pair<double, double>>>();
  camera_reading->insert(std::make_pair("Gate10-1", std::make_pair(402.281, 195.785)));
  camera_reading->insert(std::make_pair("Gate12-3", std::make_pair(55.6591, 147.801)));
  estimator.callback_cm(camera_reading, "Camera 1");
  std::cout << "\nending camera callback\n" << std::endl;

  estimator.latest_state();
  std::cout << "\nstarting odom callback\n" << std::endl;
  reading_odom->x = 0.1;
  reading_odom->y = 0.1;
  reading_odom->x_dot = 0.0;
  reading_odom->y_dot = 0.0;
  estimator.callback_odometry(reading_odom);
  std::cout << "\nodom callback ended\n" << std::endl;

  std::cout << "\nstarting imu callback\n" << std::endl;
  reading_imu->time = 2.0;
  reading_imu->x_accel = -1.0;
  reading_imu->y_accel = -1.0;
  estimator.callback_imu(reading_imu);
  std::cout << "\nending imu callback\n" << std::endl;

  std::cout << "\nstarting camera callback\n" << std::endl;
  estimator.callback_cm(camera_reading, "Camera 1");

  estimator.latest_state();
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
