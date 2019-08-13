//
// Created by jason on 6/1/19.
//

#include <StateEstimator/estimation/FactorGraph/FactorGraphEstimator.h>

using namespace alphapilot::estimator;
using namespace alphapilot;
using namespace gtsam;

// gtrue x: -49.997 y: -0.00010982 z: 0.94244 x_dot: -5.3067e-05 y_dot: -1.0736e-06 z_dot: 0.00047878, roll: 0.010406, pitch: -0.17072, yaw: -2.3388 at 7.754 Sim Time: 7.6799
/*
Factor 0: GenericProjectionFactor, z = (461, 454)
  sensor pose in body frame: R:
[
          0.707107433   0.183010728  -0.683012581;
        9.4173524e-07   0.965926267   0.258817408;
          0.707106153  -0.183012361    0.68301347
  ]
[0.171, 0.03, -0.07]';  keys = { x99 a21 }
isotropic dim=2 sigma=10
error = 12198.9744

Factor 1: GenericProjectionFactor, z = (491, 458)
  sensor pose in body frame: R:
[
          0.707107433   0.183010728  -0.683012581;
        9.4173524e-07   0.965926267   0.258817408;
          0.707106153  -0.183012361    0.68301347
  ]
[0.171, 0.03, -0.07]';  keys = { x99 a22 }
isotropic dim=2 sigma=10
error = 12198.9744

Factor 2: GenericProjectionFactor, z = (492, 485)
  sensor pose in body frame: R:
[
          0.707107433   0.183010728  -0.683012581;
        9.4173524e-07   0.965926267   0.258817408;
          0.707106153  -0.183012361    0.68301347
  ]
[0.171, 0.03, -0.07]';  keys = { x99 a23 }
isotropic dim=2 sigma=10
error = 12198.9744
Factor 3: GenericProjectionFactor, z = (463, 483)
  sensor pose in body frame: R:
[
          0.707107433   0.183010728  -0.683012581;
        9.4173524e-07   0.965926267   0.258817408;
          0.707106153  -0.183012361    0.68301347
  ]
[0.171, 0.03, -0.07]';  keys = { x99 a24 }
isotropic dim=2 sigma=10
error = 12198.9744

Factor 4: GenericProjectionFactor, z = (637, 411)
  sensor pose in body frame: R:
[
          0.707107433   0.183010728  -0.683012581;
        9.4173524e-07   0.965926267   0.258817408;
          0.707106153  -0.183012361    0.68301347
  ]
[0.171, 0.03, -0.07]';  keys = { x99 a11 }
isotropic dim=2 sigma=10
error = 12198.9744

Factor 5: GenericProjectionFactor, z = (698, 417)
  sensor pose in body frame: R:
[
          0.707107433   0.183010728  -0.683012581;
        9.4173524e-07   0.965926267   0.258817408;
          0.707106153  -0.183012361    0.68301347
  ]
[0.171, 0.03, -0.07]';  keys = { x99 a12 }
isotropic dim=2 sigma=10
error = 12198.9744
Factor 6: GenericProjectionFactor, z = (697, 482)
  sensor pose in body frame: R:
[
          0.707107433   0.183010728  -0.683012581;
        9.4173524e-07   0.965926267   0.258817408;
          0.707106153  -0.183012361    0.68301347
  ]
[0.171, 0.03, -0.07]';  keys = { x99 a13 }
isotropic dim=2 sigma=10
error = 12198.9744

Factor 7: GenericProjectionFactor, z = (637, 481)
  sensor pose in body frame: R:
[
          0.707107433   0.183010728  -0.683012581;
        9.4173524e-07   0.965926267   0.258817408;
          0.707106153  -0.183012361    0.68301347
  ]
[0.171, 0.03, -0.07]';  keys = { x99 a14 }
isotropic dim=2 sigma=10
error = 12198.9744
 */

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
  aruco_detection.id = 2;
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

  estimator.timing_callback(1.0 + starting_time);

  std::shared_ptr<ArucoDetections> aruco_reading2 = std::make_shared<ArucoDetections>();
  aruco_reading2->time = 0.1 + starting_time;
  aruco_reading2->camera = "right_left";

  ArucoDetection detection2;
  detection2.id = 1;
  detection2.pose.position.x = 3.0;
  detection2.pose.position.y = 2.2;
  detection2.pose.position.y = 1;

  detection2.points[0].x = 698;
  detection2.points[0].y = 417;

  detection2.points[1].x = 697;
  detection2.points[1].y = 482;

  detection2.points[2].x = 637;
  detection2.points[2].y = 481;

  detection2.points[3].x = 637;
  detection2.points[3].y = 481;

  aruco_reading2->detections.push_back(detection2);

  estimator.aruco_callback(aruco_reading2);

  estimator.run_optimize();

  for(int i = 1; i < 0; i++) {
    reading_imu->time = 0.1 + starting_time + 0.1 * i;
    reading_imu->x_accel = 0.0;
    reading_imu->y_accel = 0.0;
    reading_imu->z_accel = 9.81;
    reading_imu->roll_vel = 1.0;
    reading_imu->pitch_vel = 0.0;
    reading_imu->yaw_vel = 0.0;
    estimator.callback_imu(reading_imu);

    estimator.timing_callback(0.1 + starting_time + 0.1 * i);

    camera_reading->time = 0.1 + starting_time + 0.1 * i;
    estimator.run_optimize();
  }
  return 0;
}
