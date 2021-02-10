//
// Created by jason on 8/18/20.
//

#include <ros/ros.h>
#include "FactorGraphEstimatorNode.h"

FactorGraphEstimatorNode::FactorGraphEstimatorNode() {
  ros::NodeHandle nh;
  ros::NodeHandle pNh("~");
  std::string config_file;
  pNh.param<std::string>("config_file", config_file, "config.yaml");

  estimator_ = std::make_unique<estimator::FactorGraphEstimator>(config_file);

  // setup subs and pubs
  std::string gps_topic;
  pNh.param<std::string>("gps_topic", gps_topic, "/gpsRoverStatus");
  gps_sub_ = nh.subscribe(gps_topic, 10, &FactorGraphEstimatorNode::gpsCallback, this);

  std::string imu_topic;
  pNh.param<std::string>("imu_topic", imu_topic, "/imu/imu");
  imu_sub_ = nh.subscribe(imu_topic, 100, &FactorGraphEstimatorNode::imuCallback, this);

  std::string aruco_topic;
  pNh.param<std::string>("aruco_topic", aruco_topic, "/left_camera/image_raw/aruco_detections");
  aruco_sub_ = nh.subscribe(aruco_topic, 10, &FactorGraphEstimatorNode::arucoCallback, this);

  std::string camera_info_topic;
  pNh.param<std::string>("camera_info_topic", camera_info_topic, "/left_camera/camera_info");
  aruco_sub_ = nh.subscribe(camera_info_topic, 10, &FactorGraphEstimatorNode::cameraInfoCallback, this);

  odom_pub_ = nh.advertise<nav_msgs::Odometry>("state", 10);

  odom_pub_timer_ = pNh.createTimer(ros::Duration(1.0/200), &FactorGraphEstimatorNode::pubOdom, this);
}

void FactorGraphEstimatorNode::gpsCallback(sensor_msgs::NavSatFixConstPtr msg) {
  estimator_->GpsCallback(msg);
}

void FactorGraphEstimatorNode::imuCallback(sensor_msgs::ImuConstPtr msg) {
  estimator_->CallbackImu(msg);
}

void FactorGraphEstimatorNode::arucoCallback(autorally_estimation::ArucoDetectionsConstPtr msg) {
  estimator_->ArucoCallback(msg);
}

void FactorGraphEstimatorNode::cameraInfoCallback(sensor_msgs::CameraInfoConstPtr msg) {
  //estimator_->TimingCallback(msg->header.stamp.toSec());
}

void FactorGraphEstimatorNode::pubOdom(const ros::TimerEvent& event) {
  nav_msgs::Odometry state = estimator_->LatestState(true);
  odom_pub_.publish(state);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "FactorGraphEstimatorNode");

  FactorGraphEstimatorNode node;

  ros::spin();
}
