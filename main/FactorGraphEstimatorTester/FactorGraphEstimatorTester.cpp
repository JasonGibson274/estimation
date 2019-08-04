//
// Created by jason on 6/1/19.
//

#include <StateEstimator/estimation/FactorGraph/FactorGraphEstimator.h>

using namespace alphapilot::estimator;
using namespace alphapilot;
using namespace gtsam;

int main() {
  FactorGraphEstimator estimator("/home/jgibson37/Documents/alpha_pilot/estimation/config/configTester.yaml", "/home/jgibson37/Documents/alpha_pilot/estimation/cmake-build-debug/2019_412");
  std::cout << "\ninit ended\n" << std::endl;

  std::cout << "\nstarting imu callback\n" << std::endl;
  std::shared_ptr<IMU_readings> reading_imu = std::make_shared<IMU_readings>();
  double starting_time = 518942580.5631;
  reading_imu->time = 0.0 + starting_time;
  reading_imu->x_accel = 0.0;
  reading_imu->y_accel = 0.0;
  reading_imu->z_accel = 9.81;
  reading_imu->roll_vel = 0.0;
  reading_imu->pitch_vel = 0.0;
  reading_imu->yaw_vel = 0.0;
  estimator.callback_imu(reading_imu);

  reading_imu->time = 0.1 + starting_time;
  estimator.callback_imu(reading_imu);
  /*

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
  */
  std::cout << "\nending imu callback\n" << std::endl;

  std::cout << "\nnew position after IMU callbacks:" << estimator.latest_state(false) << std::endl;

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

  std::cout << "\nstarting timing callback\n" << std::endl;
  estimator.timing_callback(0.1 + starting_time);
  std::cout << "\nending timing callback\n" << std::endl;

  std::cout << "\nstarting ARUCO detection callback" << std::endl;
  std::shared_ptr<alphapilot::ArucoDetections> aruco_reading =
          std::make_shared<alphapilot::ArucoDetections>();
  aruco_reading->camera = "left_left";
  aruco_reading->time = 0.1 + starting_time;

  ArucoDetection aruco_detection;
  aruco_detection.id = 1;
  aruco_detection.pose.position.x = 1.0;
  aruco_detection.pose.position.y = 1.0;
  aruco_detection.pose.position.z = 1.0;

  aruco_detection.pose.orientation.w = 1.0;
  aruco_detection.pose.orientation.x = 0.0;
  aruco_detection.pose.orientation.y = 0.0;
  aruco_detection.pose.orientation.z = 0.0;
  aruco_reading->detections.push_back(aruco_detection);

  estimator.aruco_callback(aruco_reading);

  // TODO pose should be different based off of reference frame
  aruco_reading->camera = "right_right";

  estimator.aruco_callback(aruco_reading);
  std::cout << "\nending ARUCO detection callback" << std::endl;

  std::cout << "\nstarting camera callback\n" << std::endl;
  std::shared_ptr<alphapilot::GateDetections> camera_reading = std::make_shared<alphapilot::GateDetections>();
  camera_reading->time = 0.1 + starting_time;
  camera_reading->camera_name = "left_left";

  GateDetection gate_detection1;
  gate_detection1.gate = "10";
  gate_detection1.type = "1";
  gate_detection1.x = 402.28125;
  gate_detection1.y = 195.784973145;
  camera_reading->landmarks.push_back(gate_detection1);

  GateDetection gate_detection2;
  gate_detection2.gate = "12";
  gate_detection2.type = "3";
  gate_detection2.x = 55.6590881348;
  gate_detection2.y = 147.801269531;
  camera_reading->landmarks.push_back(gate_detection2);

  estimator.callback_cm(camera_reading);
  std::cout << "\nending camera callback\n" << std::endl;

  estimator.latest_state();
  estimator.run_optimize();
  std::cout << "optimized" << std::endl;

  for(int i = 1; i < 100; i++) {
    reading_imu->time = 0.1 + starting_time + 0.1 * i;
    reading_imu->x_accel = 0.0;
    reading_imu->y_accel = 0.0;
    reading_imu->z_accel = 9.81;
    reading_imu->roll_vel = 0.0;
    reading_imu->pitch_vel = 0.0;
    reading_imu->yaw_vel = 0.0;
    estimator.callback_imu(reading_imu);

    estimator.timing_callback(0.1 + starting_time + 0.1 * i);

    camera_reading->time = 0.1 + starting_time + 0.1 * i;
    estimator.callback_cm(camera_reading);
    estimator.run_optimize();
  }
  return 0;
}
