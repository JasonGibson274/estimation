//
// Created by jason on 6/1/19.
//

#include <StateEstimator/estimation/FactorGraph/FactorGraphEstimator.h>

using namespace alphapilot::estimator;
using namespace alphapilot;
using namespace gtsam;

int main() {
  FactorGraphEstimator estimator("/home/jason/Documents/alpha_pilot/estimation/config/configTester.yaml", "/home/jason/Documents/alpha_pilot/estimation/cmake-build-debug/2019_412");
  //estimator.resetGraph(config.priorConfig.initial_state);
  std::cout << "\ninit ended\n" << std::endl;

  double starting_time = 10000;

  std::cout << "\nstarting odom callback\n" << std::endl;
  std::shared_ptr<drone_state> reading_odom = std::make_shared<drone_state>();
  reading_odom->z = 1.0;
  reading_odom->y = 0.5;
  reading_odom->x = 0.5;
  reading_odom->x_dot = 1.0;
  reading_odom->y_dot = 1.0;
  reading_odom->qx = 0.0;
  reading_odom->qy = 0.0;
  reading_odom->qz = 0.0;
  reading_odom->qw = 1.0;
  estimator.callback_odometry(reading_odom);
  reading_odom->y = -0.5;
  reading_odom->x = -0.5;
  estimator.callback_odometry(reading_odom);
  reading_odom->y = -1.0;
  reading_odom->x = -1.0;
  estimator.callback_odometry(reading_odom);
  reading_odom->y = 0.5;
  reading_odom->x = 0.5;
  estimator.callback_odometry(reading_odom);
  std::cout << "\nodom callback ended\n" << std::endl;


  std::cout << "\nstarting imu callback\n" << std::endl;
  std::shared_ptr<IMU_readings> reading_imu = std::make_shared<IMU_readings>();
  reading_imu->time = 0.0 + starting_time;
  reading_imu->x_accel = 1.0;
  reading_imu->y_accel = 1.0;
  reading_imu->z_accel = 9.81;
  reading_imu->roll_vel = 0.0;
  reading_imu->pitch_vel = 0.0;
  reading_imu->yaw_vel = 0.0;
  estimator.callback_imu(reading_imu);

  reading_imu->time = 0.1 + starting_time;
  estimator.callback_imu(reading_imu);

  reading_imu->time = 0.2 + starting_time;
  estimator.callback_imu(reading_imu);

  reading_imu->time = 0.3 + starting_time;
  estimator.callback_imu(reading_imu);

  reading_imu->time = 0.4 + starting_time;
  estimator.callback_imu(reading_imu);

  reading_imu->time = 0.5 + starting_time;
  estimator.callback_imu(reading_imu);

  reading_imu->time = 0.6 + starting_time;
  estimator.callback_imu(reading_imu);

  reading_imu->time = 0.7 + starting_time;
  estimator.callback_imu(reading_imu);

  reading_imu->time = 0.8 + starting_time;
  estimator.callback_imu(reading_imu);

  reading_imu->time = 0.9 + starting_time;
  estimator.callback_imu(reading_imu);

  reading_imu->time = 1.0 + starting_time;
  estimator.callback_imu(reading_imu);
  std::cout << "\nending imu callback\n" << std::endl;


  std::cout << "\nstarting camera callback\n" << std::endl;
  std::shared_ptr<std::map<std::string, std::pair<double, double>>> camera_reading = std::make_shared<std::map<std::string, std::pair<double, double>>>();
  camera_reading->insert(std::make_pair("8_1", std::make_pair(402.281, 195.785)));
  camera_reading->insert(std::make_pair("3_478", std::make_pair(55.6591, 147.801)));
  estimator.callback_cm(camera_reading, "left_left");
  estimator.callback_cm(camera_reading, "right_right");
  std::cout << "\nending camera callback\n" << std::endl;

  estimator.latest_state();
  estimator.run_optimize();
  std::cout << "optimized" << std::endl;

  std::cout << "\nstarting imu callback\n" << std::endl;
  reading_imu->time = 3.0 + starting_time;
  reading_imu->x_accel = -1.0;
  reading_imu->y_accel = -1.0;
  estimator.callback_imu(reading_imu);
  std::cout << "\nending imu callback\n" << std::endl;

  std::cout << "\nstarting odom callback\n" << std::endl;
  reading_odom->x = 0.5;
  reading_odom->y = 0.5;
  reading_odom->x_dot = -1.0;
  reading_odom->y_dot = -1.0;
  estimator.callback_odometry(reading_odom);
  std::cout << "\nodom callback ended\n" << std::endl;

  std::cout << "\nstarting camera callback\n" << std::endl;
  estimator.callback_cm(camera_reading, "left_left");
  estimator.callback_cm(camera_reading, "right_right");

  estimator.latest_state(true);
  std::cout << "\n\noptimization time = " << estimator.get_optimization_time() << std::endl;
  return 0;

}
