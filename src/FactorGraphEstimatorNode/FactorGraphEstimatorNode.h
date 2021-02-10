//
// Created by jason on 9/26/20.
//

#ifndef ACDSSTATEESTIMATION_FACTORGRAPHESTIMATORNODE_H
#define ACDSSTATEESTIMATION_FACTORGRAPHESTIMATORNODE_H

#include <StateEstimator/FactorGraph/FactorGraphEstimator.h>
#include <ros/ros.h>

class FactorGraphEstimatorNode {
  ros::Subscriber gps_sub_;
  ros::Subscriber imu_sub_;
  ros::Subscriber aruco_sub_;
  ros::Subscriber camera_info_sub_;

  ros::Publisher odom_pub_;

  std::unique_ptr<estimator::FactorGraphEstimator> estimator_;

  ros::Timer odom_pub_timer_;
  ros::Timer optimize_timer_;
public:
  FactorGraphEstimatorNode();

  void gpsCallback(sensor_msgs::NavSatFixConstPtr msg);
  void imuCallback(sensor_msgs::ImuConstPtr msg);
  void arucoCallback(autorally_estimation::ArucoDetectionsConstPtr msg);
  void cameraInfoCallback(sensor_msgs::CameraInfoConstPtr msg);

  void pubOdom(const ros::TimerEvent& event);
};

#endif //ACDSSTATEESTIMATION_FACTORGRAPHESTIMATORNODE_H
