//
// Created by jason on 6/1/19.
//

#include <StateEstimator/estimation/FactorGraph/FactorGraphEstimator.h>

using namespace alphapilot::estimator;
using namespace alphapilot;
using namespace gtsam;


void updatePoseAndVelTest(gtsam::Pose3& fixed_pose, gtsam::Vector3& fixed_vel,
                                            const gtsam::Pose3& last_fixed_pose, const gtsam::Vector3& last_fixed_vel,
                                            const gtsam::Pose3& last_guess_pose, const gtsam::Vector3& last_guess_vel,
                                            const gtsam::Pose3& guess_pose, const gtsam::Vector3& guess_vel,
                                            const double dt) {

  Vector3 delta_v = guess_vel - last_guess_vel;
  std::cout << "delta_v = " << delta_v.transpose() << std::endl;
  fixed_vel =  last_fixed_vel + (delta_v) * dt;

  Vector3 fixed_translation = last_fixed_pose.translation().vector() + (last_fixed_vel * dt) +
                              0.5 * delta_v * dt;

  Rot3 delta_r = traits<gtsam::Rot3>::Between(last_guess_pose.rotation(), guess_pose.rotation());
  std::cout << "delta_r = " << delta_r.ypr().transpose() << std::endl;
  Rot3 fixed_rot = last_fixed_pose.rotation() * delta_r;
  fixed_pose = Pose3(fixed_rot, fixed_translation);
}


int main() {
  //FactorGraphEstimator estimator("/home/jason/Documents/alpha_pilot/estimation/config/configTester.yaml", "/home/jgibson37/Documents/alpha_pilot/estimation/cmake-build-debug/2019_412");
  //std::cout << "\ninit ended\n" << std::endl;
  double pi = M_PI;

  Pose3 fixed_pose;
  Vector3 fixed_vel;
  Pose3 last_fixed_pose = Pose3(Rot3::Ypr(pi/2, 0, 0), Point3(1,0,0));
  Vector3 last_fixed_vel = Vector3(0,0,-1);
  Pose3 last_guess_pose = Pose3(Rot3::Ypr(0,0,0), Point3(0,0,0));
  Vector3 last_guess_vel = Vector3(0,0,0);
  Pose3 guess_pose = Pose3(Rot3::Ypr(pi/2, 0, 0), Point3(0,0,1.0));
  Vector3 guess_vel = Vector3(0,0,2);
  double dt = 1;
  updatePoseAndVelTest(fixed_pose, fixed_vel, last_fixed_pose, last_fixed_vel,
          last_guess_pose, last_guess_vel, guess_pose, guess_vel, dt);
  std::cout << "fixed pose\n" << fixed_pose << std::endl;
  std::cout << "euler should be:\n" << Rot3::Ypr(pi,0,0).ypr().transpose() << std::endl;
  std::cout << "euler angles:\n" << fixed_pose.rotation().ypr().transpose() << std::endl;
  std::cout << "fixed vel\n" << fixed_vel.transpose() << std::endl;
/*
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

  // TODO verify that the state has changed

  std::cout << "\nending imu callback\n" << std::endl;

  std::cout << "\nnew position after IMU callbacks:" << estimator.latest_state(false) << std::endl;
   */

  /*
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
   */
  /*
  std::cout << "\nstarting timing callback\n" << std::endl;
  estimator.timing_callback(0.1 + starting_time);
  std::cout << "\nending timing callback\n" << std::endl;

  std::cout << "\nstarting optimization\n" << std::endl;
  estimator.run_optimize();
  std::cout << "\nending optimization\n" << std::endl;
   */

  /*
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
   */
  /*

  std::cout << "\nstarting gate detection callback\n" << std::endl;
  std::shared_ptr<alphapilot::GateDetections> camera_reading = std::make_shared<alphapilot::GateDetections>();
  camera_reading->time = 0.1 + starting_time;
  camera_reading->camera_name = "left";

  // TODO insert landmarks priors

  GateDetection gate_detection1;
  gate_detection1.gate = "dgjsklds";
  Subfeature subfeature1;
  subfeature1.type = "1";
  subfeature1.x = 402.28125;
  subfeature1.y = 195.784973145;
  gate_detection1.subfeatures.push_back(subfeature1);

  gate_detection1.pose.position.x = 18;
  gate_detection1.pose.position.y = 15;
  gate_detection1.pose.position.z = 9;

  camera_reading->landmarks.push_back(gate_detection1);

  GateDetection gate_detection2;
  gate_detection2.gate = "gjksalh;";
  Subfeature subfeature2;
  subfeature2.type = "3";
  subfeature2.x = 55.6590881348;
  subfeature2.y = 147.801269531;
  gate_detection2.subfeatures.push_back(subfeature2);

  gate_detection2.pose.position.x = 18;
  gate_detection2.pose.position.y = 3;
  gate_detection2.pose.position.z = 7;

  camera_reading->landmarks.push_back(gate_detection2);

  estimator.callback_cm(camera_reading);
  std::cout << "\nending camera callback\n" << std::endl;

  estimator.latest_state();
  estimator.run_optimize();
  std::cout << "optimized" << std::endl;

  for(double i = 0.2; i <= 1.0; i+=0.1) {
    reading_imu->time = i + starting_time;
    estimator.callback_imu(reading_imu);
    estimator.timing_callback(i + starting_time);
  }
   */
  /*
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
   */

  //estimator.run_optimize();
  return 0;
}
