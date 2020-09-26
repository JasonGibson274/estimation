#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <random>
#include <algorithm>
#include <numeric>

#include <StateEstimator/FactorGraph/FactorGraphEstimator.h>
#include <autorally_factor_graph_test_config.h>

using namespace estimator;

class FactorGraphEstimatorTester : public FactorGraphEstimator {

};

void testConstructor(std::string config_file, bool debug, std::string optimization_method) {
  FactorGraphEstimator estimator(config_file);

  // general
  EXPECT_NE(estimator.getCurrentStateGuess(), nullptr);
  EXPECT_NE(estimator.getCurrentIncrementalGraph(), nullptr);
  EXPECT_EQ(estimator.getDebugMode(), debug);

  // prior
  PriorConfig prior_config = estimator.getPriorConfig();
  EXPECT_DOUBLE_EQ(prior_config.state.x(), 0.0);
  EXPECT_DOUBLE_EQ(prior_config.state.y(), 0.0);
  EXPECT_DOUBLE_EQ(prior_config.state.z(), 1.0);
  gtsam::Rot3 rotation(1.0, 0.0, 0.0, 0.0);
  EXPECT_TRUE(prior_config.state.rotation().equals(rotation));
  EXPECT_DOUBLE_EQ(prior_config.velocity.x(), 0);
  EXPECT_DOUBLE_EQ(prior_config.velocity.y(), 0);
  EXPECT_DOUBLE_EQ(prior_config.velocity.z(), 0);
  EXPECT_DOUBLE_EQ(prior_config.initial_vel_noise, 0.01);
  EXPECT_DOUBLE_EQ(prior_config.initial_bias_noise, 0.1);

  EXPECT_DOUBLE_EQ(prior_config.initial_pose_noise[0], 0.5); // roll
  EXPECT_DOUBLE_EQ(prior_config.initial_pose_noise[1], 0.5); // pitch
  EXPECT_DOUBLE_EQ(prior_config.initial_pose_noise[2], 0.1); // yaw
  EXPECT_DOUBLE_EQ(prior_config.initial_pose_noise[3], 0.15); // x
  EXPECT_DOUBLE_EQ(prior_config.initial_pose_noise[4], 0.15); // y
  EXPECT_DOUBLE_EQ(prior_config.initial_pose_noise[5], 0.15); // z

  // ISAM params
  gtsam::ISAM2Params isam_params = estimator.getISAMParams();
  EXPECT_DOUBLE_EQ(boost::get<double>(isam_params.relinearizeThreshold), 0.01);
  EXPECT_EQ(isam_params.relinearizeSkip, 1);
  EXPECT_EQ(isam_params.enablePartialRelinearizationCheck, false);
  EXPECT_EQ(isam_params.enableRelinearization, true);
  EXPECT_EQ(isam_params.cacheLinearizedFactors, false);
  EXPECT_EQ(isam_params.findUnusedFactorSlots, false);
  EXPECT_EQ(isam_params.enableDetailedResults, debug);
  EXPECT_EQ(isam_params.evaluateNonlinearError, debug);
  if (isam_params.optimizationParams.type() == typeid(gtsam::ISAM2GaussNewtonParams)) {
    gtsam::ISAM2GaussNewtonParams params = boost::get<gtsam::ISAM2GaussNewtonParams>(isam_params.optimizationParams);
    EXPECT_DOUBLE_EQ(params.wildfireThreshold, 0.001);
  } else if (isam_params.optimizationParams.type() == typeid(gtsam::ISAM2DoglegParams)) {
    gtsam::ISAM2DoglegParams params = boost::get<gtsam::ISAM2DoglegParams>(isam_params.optimizationParams);
    EXPECT_EQ(params.verbose, debug);
    EXPECT_DOUBLE_EQ(params.wildfireThreshold, 1e-5);
    EXPECT_DOUBLE_EQ(params.initialDelta, 1.0);
  }

  // check cameras
  EXPECT_DOUBLE_EQ(estimator.getPairingThreshold(), 0.02);
  std::map<std::string, GtsamCamera> camera_map = estimator.getCameraMap();
  for(std::pair<std::string, GtsamCamera> pair : camera_map) {
    if(pair.first == "left") {
      EXPECT_DOUBLE_EQ(pair.second.k->fx(), 548.4088134765625);
      EXPECT_DOUBLE_EQ(pair.second.k->fy(), 548.4088134765625);
      EXPECT_DOUBLE_EQ(pair.second.k->skew(), 0.0);
      EXPECT_DOUBLE_EQ(pair.second.k->px(), 512);
      EXPECT_DOUBLE_EQ(pair.second.k->py(), 384);

      EXPECT_DOUBLE_EQ(pair.second.transform.x(), 0.0);
      EXPECT_DOUBLE_EQ(pair.second.transform.y(), 0.0);
      EXPECT_DOUBLE_EQ(pair.second.transform.z(), 0.0);
      EXPECT_TRUE(pair.second.transform.rotation().equals(gtsam::Rot3(-0.5, 0.5, -0.5, 0.5)));
    } else if(pair.first == "right") {
      EXPECT_DOUBLE_EQ(pair.second.k->fx(), 548.4088134765625);
      EXPECT_DOUBLE_EQ(pair.second.k->fy(), 548.4088134765625);
      EXPECT_DOUBLE_EQ(pair.second.k->skew(), 0.0);
      EXPECT_DOUBLE_EQ(pair.second.k->px(), 512);
      EXPECT_DOUBLE_EQ(pair.second.k->py(), 384);

      EXPECT_DOUBLE_EQ(pair.second.transform.x(), 0.2);
      EXPECT_DOUBLE_EQ(pair.second.transform.y(), 0.0);
      EXPECT_DOUBLE_EQ(pair.second.transform.z(), 0.0);
      EXPECT_TRUE(pair.second.transform.rotation().equals(gtsam::Rot3(-0.5, 0.5, -0.5, 0.5)));
    }
  }

  // check projection

// check aruco

  // check IMU
  EXPECT_EQ(estimator.getVerboseImu(), false);
  EXPECT_EQ(estimator.getUseImuFactors(), true);
  EXPECT_EQ(estimator.getUseImuBias(), true);
  EXPECT_EQ(estimator.getInvertImuStatus()[0], false);
  EXPECT_EQ(estimator.getInvertImuStatus()[1], false);
  EXPECT_EQ(estimator.getInvertImuStatus()[2], false);
  EXPECT_EQ(estimator.getImuBiasIncr(), 3);
  EXPECT_EQ(estimator.getImuBiasIndex(), 0);
  gtsam::noiseModel::Diagonal::shared_ptr bias_noise = estimator.getImuBiasNoise();
  for(int i = 0; i < 6; i++) {
    EXPECT_DOUBLE_EQ(bias_noise->sigmas()[i], sqrt(0.1));
  }
  boost::shared_ptr<gtsam::PreintegrationCombinedParams> combined_params =
          boost::static_pointer_cast<gtsam::PreintegrationCombinedParams>(estimator.getImuMeasurementsObject().params());
  gtsam::Matrix3 acc_cov = combined_params->getAccelerometerCovariance();
  gtsam::Matrix3 int_cov = combined_params->getIntegrationCovariance();
  gtsam::Matrix3 gyro_cov = combined_params->getGyroscopeCovariance();
  for(int i = 0; i < 3; i++) {
    for(int j = 0; j < 3; j++) {
      if(i == j) {
        EXPECT_DOUBLE_EQ(acc_cov(i,j), 1.0);
        EXPECT_DOUBLE_EQ(gyro_cov(i,j), pow(0.5, 2));
        EXPECT_DOUBLE_EQ(int_cov(i,j), 1e-4);
      } else {
        EXPECT_DOUBLE_EQ(acc_cov(i,j), 0.0);
        EXPECT_DOUBLE_EQ(gyro_cov(i,j), 0.0);
        EXPECT_DOUBLE_EQ(int_cov(i,j), 0.0);
      }
    }
  }
  gtsam::Vector3 gravity = combined_params->getGravity();
  EXPECT_DOUBLE_EQ(gravity[0], 0.0);
  EXPECT_DOUBLE_EQ(gravity[1], 0.0);
  EXPECT_DOUBLE_EQ(gravity[2], -9.81);
  // TODO what are these??
  gtsam::Matrix3 acc_bias_cov = combined_params->getBiasAccCovariance();
  gtsam::Matrix3 bias_omega_cov = combined_params->getBiasOmegaCovariance();
  for(int i = 0; i < 3; i++) {
    for(int j = 0; j < 3; j++) {
      if(i == j) {
        EXPECT_DOUBLE_EQ(acc_bias_cov(i,j), pow(0.1, 2));
        EXPECT_DOUBLE_EQ(bias_omega_cov(i,j), pow(0.1, 2));
      } else {
        EXPECT_DOUBLE_EQ(acc_bias_cov(i,j), 0.0);
        EXPECT_DOUBLE_EQ(bias_omega_cov(i,j), 0.0);
      }
    }
  }
  gtsam::Matrix6 acc_bias_omega = combined_params->getBiasAccOmegaInt();
  for(int i = 0; i < 6; i++) {
    for(int j = 0; j < 6; j++) {
      if(i == j) {
        EXPECT_DOUBLE_EQ(acc_bias_omega(i,j), 0.1);
      } else {
        EXPECT_DOUBLE_EQ(acc_bias_omega(i,j), 0.0);
      }
    }
  }

  // pose factor
  EXPECT_EQ(estimator.getUsePoseFactor(), false);
  gtsam::noiseModel::Diagonal::shared_ptr odom_vel_noise = estimator.getOdometryVelNoise();
  for(int i = 0; i < 3; i++) {
    EXPECT_DOUBLE_EQ(odom_vel_noise->sigmas()[i], 5.0);
  }
  gtsam::noiseModel::Diagonal::shared_ptr odom_pose_noise = estimator.getOdometryPoseNoise();
  // for the x,y,z noise
  for(int i = 0; i < 3; i++) {
    EXPECT_DOUBLE_EQ(odom_pose_noise->sigmas()[i], 1.0);
  }
  // for the r,p,y noise
  for(int i = 3; i < 6; i++) {
    EXPECT_DOUBLE_EQ(odom_pose_noise->sigmas()[i], 5.0);
  }

  // smart projection
  EXPECT_EQ(estimator.getUseSmartProjectionFactor(), true);
  gtsam::SmartProjectionParams smart_params = estimator.getSmartProjectionParams();
  EXPECT_EQ(smart_params.getDegeneracyMode(), gtsam::DegeneracyMode::ZERO_ON_DEGENERACY);
  EXPECT_DOUBLE_EQ(smart_params.getTriangulationParameters().landmarkDistanceThreshold, -1);
  EXPECT_DOUBLE_EQ(smart_params.getTriangulationParameters().dynamicOutlierRejectionThreshold, -1);
  EXPECT_EQ(smart_params.throwCheirality, false);

  gtsam::noiseModel::Isotropic::shared_ptr smart_noise = estimator.getSmartProjectionNoise();
  EXPECT_DOUBLE_EQ(smart_noise->sigma(), 60.0);

  // TODO GPS factor
  EXPECT_EQ(estimator.getUseGpsFactor(), true);
  EXPECT_EQ(estimator.getFirstFix(), true);
  EXPECT_DOUBLE_EQ(estimator.getImuToGps().x(), -0.37);
  EXPECT_DOUBLE_EQ(estimator.getImuToGps().y(), 0.0);
  EXPECT_DOUBLE_EQ(estimator.getImuToGps().z(), -0.06);
  EXPECT_DOUBLE_EQ(estimator.getMaxGpsError(), 5.0);
  EXPECT_EQ(estimator.getUseFixedOrigin(), false);

  // TODO check all noises and what they actually mean
  std::shared_ptr<gtsam::NonlinearFactorGraph> current_graph = estimator.getCurrentIncrementalGraph();
  std::shared_ptr<gtsam::Values> current_state = estimator.getCurrentStateGuess();
  int index = estimator.getCurrentIndex();

}

TEST(FactorGraphEstimatorLibSimple, Constructor) {
  testConstructor(tests::test_config_file, true, "GN");
}

TEST(FactorGraphEstimatorLibSimple, propagateImu) {
  FactorGraphEstimator estimator(tests::test_config_file);

  gtsam::Rot3 rotation(1.0, 0.0, 0.0, 0.0);
  gtsam::Point3 position(0, 0, 1.0);
  gtsam::Pose3 pose(rotation, position);
  gtsam::Vector3 vel(0.0, 0.0, 0.0);

  gtsam::Vector3 acc(0.0, 0.0, 9.81);
  gtsam::Vector3 angular_vel(0.0, 0.0, 0.0);
  double dt = 0.1;

  gtsam::PreintegratedImuMeasurements preintegrator = estimator.getImuMeasurementsObject();
  preintegrator.integrateMeasurement(acc, angular_vel, dt);

  estimator.PropagateImu(pose, vel, preintegrator);

  EXPECT_DOUBLE_EQ(pose.x(), 0.0);
  EXPECT_DOUBLE_EQ(pose.y(), 0.0);
  EXPECT_DOUBLE_EQ(pose.z(), 1.0);

  EXPECT_DOUBLE_EQ(pose.rotation().quaternion()[0], 1.0);
  EXPECT_DOUBLE_EQ(pose.rotation().quaternion()[1], 0.0);
  EXPECT_DOUBLE_EQ(pose.rotation().quaternion()[2], 0.0);
  EXPECT_DOUBLE_EQ(pose.rotation().quaternion()[3], 0.0);

  EXPECT_DOUBLE_EQ(vel.x(), 0.0);
  EXPECT_DOUBLE_EQ(vel.y(), 0.0);
  EXPECT_NEAR(vel.z(), 0.0, 1e-6);

  acc = gtsam::Vector3(1.0, 1.0, 9.81);
  angular_vel = gtsam::Vector3(0.0, 0.0, 0.0);
  preintegrator.integrateMeasurement(acc, angular_vel, dt);

  estimator.PropagateImu(pose, vel, preintegrator);

  EXPECT_DOUBLE_EQ(pose.x(), 0.005);
  EXPECT_DOUBLE_EQ(pose.y(), 0.005);
  EXPECT_DOUBLE_EQ(pose.z(), 1.0);

  EXPECT_DOUBLE_EQ(pose.rotation().quaternion()[0], 1.0);
  EXPECT_DOUBLE_EQ(pose.rotation().quaternion()[1], 0.0);
  EXPECT_DOUBLE_EQ(pose.rotation().quaternion()[2], 0.0);
  EXPECT_DOUBLE_EQ(pose.rotation().quaternion()[3], 0.0);

  EXPECT_DOUBLE_EQ(vel.x(), 0.1);
  EXPECT_DOUBLE_EQ(vel.y(), 0.1);
  EXPECT_NEAR(vel.z(), 0.0, 1e-6);

  /*
  acc = gtsam::Vector3(1.0, -1.0, 9.81);
  angular_vel = gtsam::Vector3(1.0, 0.0, -1.0);
  preintegrator.integrateMeasurement(acc, angular_vel, dt);

  estimator.PropagateImu(pose, vel, preintegrator);

  EXPECT_DOUBLE_EQ(pose.x(), 0.005);
  EXPECT_DOUBLE_EQ(pose.y(), 0.005);
  EXPECT_DOUBLE_EQ(pose.z(), 1.0);

  EXPECT_DOUBLE_EQ(pose.rotation().quaternion()[0], 1.0);
  EXPECT_DOUBLE_EQ(pose.rotation().quaternion()[1], 0.0);
  EXPECT_DOUBLE_EQ(pose.rotation().quaternion()[2], 0.0);
  EXPECT_DOUBLE_EQ(pose.rotation().quaternion()[3], 0.0);

  EXPECT_DOUBLE_EQ(vel.x(), 0.1);
  EXPECT_DOUBLE_EQ(vel.y(), 0.1);
  EXPECT_NEAR(vel.z(), 0.0, 1e-6);
   */
}

TEST(FactorGraphEstimatorLibSimple, IMUCallback) {
  FactorGraphEstimator estimator(tests::test_config_file);

  std::shared_ptr<sensor_msgs::Imu> imu_data = std::make_shared<sensor_msgs::Imu>();
  imu_data->header.stamp = ros::Time(100);

  imu_data->linear_acceleration.x = 1.0;
  imu_data->linear_acceleration.y = 1.0;
  imu_data->linear_acceleration.z = 9.81;

  imu_data->angular_velocity.x = 1.0;
  imu_data->angular_velocity.y = 0.0;
  imu_data->angular_velocity.z = 0.0;

  estimator.CallbackImu(imu_data);
  nav_msgs::Odometry state = estimator.LatestState();
  EXPECT_DOUBLE_EQ(estimator.getLastImuTime(), 100.0);
  EXPECT_EQ(estimator.getPositionUpdate(), false);

  EXPECT_DOUBLE_EQ(state.pose.pose.position.x, 0.0);
  EXPECT_DOUBLE_EQ(state.pose.pose.position.y, 0.0);
  EXPECT_DOUBLE_EQ(state.pose.pose.position.z, 1.0);

  EXPECT_DOUBLE_EQ(state.pose.pose.orientation.w, 1.0);
  EXPECT_DOUBLE_EQ(state.pose.pose.orientation.x, 0.0);
  EXPECT_DOUBLE_EQ(state.pose.pose.orientation.y, 0.0);
  EXPECT_DOUBLE_EQ(state.pose.pose.orientation.z, 0.0);

  EXPECT_DOUBLE_EQ(state.twist.twist.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(state.twist.twist.linear.y, 0.0);
  EXPECT_DOUBLE_EQ(state.twist.twist.linear.z, 0.0);

  EXPECT_DOUBLE_EQ(state.twist.twist.angular.x, 0.0);
  EXPECT_DOUBLE_EQ(state.twist.twist.angular.y, 0.0);
  EXPECT_DOUBLE_EQ(state.twist.twist.angular.z, 0.0);

  imu_data->header.stamp = ros::Time(101);
  estimator.CallbackImu(imu_data);
  EXPECT_DOUBLE_EQ(estimator.getLastImuTime(), 101.0);
  EXPECT_EQ(estimator.getPositionUpdate(), true);

  state = estimator.LatestState();
  EXPECT_DOUBLE_EQ(state.pose.pose.position.x, 0.5);
  EXPECT_DOUBLE_EQ(state.pose.pose.position.y, 0.5);
  EXPECT_DOUBLE_EQ(state.pose.pose.position.z, 1.0);

  EXPECT_DOUBLE_EQ(state.pose.pose.orientation.w, 0.87758256189037276);
  EXPECT_DOUBLE_EQ(state.pose.pose.orientation.x, 0.47942553860420295);
  EXPECT_DOUBLE_EQ(state.pose.pose.orientation.y, 0.0);
  EXPECT_DOUBLE_EQ(state.pose.pose.orientation.z, 0.0);

  EXPECT_DOUBLE_EQ(state.twist.twist.linear.x, 1.0);
  EXPECT_DOUBLE_EQ(state.twist.twist.linear.y, 1.0);
  EXPECT_DOUBLE_EQ(state.twist.twist.linear.z, 0.0);

  EXPECT_DOUBLE_EQ(state.twist.twist.angular.x, 1.0);
  EXPECT_DOUBLE_EQ(state.twist.twist.angular.y, 0.0);
  EXPECT_DOUBLE_EQ(state.twist.twist.angular.z, 0.0);
}

TEST(FactorGraphEstimatorLibSimple, AddImuFactorNoPosition) {
  FactorGraphEstimator estimator(tests::test_config_file);

  std::shared_ptr<sensor_msgs::Imu> imu_data = std::make_shared<sensor_msgs::Imu>();
  imu_data->header.stamp = ros::Time(100);

  imu_data->linear_acceleration.x = 1.0;
  imu_data->linear_acceleration.y = 1.0;
  imu_data->linear_acceleration.z = 9.81;

  imu_data->angular_velocity.x = 1.0;
  imu_data->angular_velocity.y = 0.0;
  imu_data->angular_velocity.z = 0.0;
  estimator.CallbackImu(imu_data);

  estimator.AddImuFactor();

  std::map<int, double> time_map = estimator.getTimeMap();
  EXPECT_EQ(estimator.getCurrentIndex(), 0);
  EXPECT_EQ(time_map.size(), 0);
  // TODO check error message
}


TEST(FactorGraphEstimatorLibSimple, AddImuFactor) {
  FactorGraphEstimator estimator(tests::test_config_file);

  std::shared_ptr<sensor_msgs::Imu> imu_data = std::make_shared<sensor_msgs::Imu>();
  imu_data->header.stamp = ros::Time(100);

  imu_data->linear_acceleration.x = 1.0;
  imu_data->linear_acceleration.y = 1.0;
  imu_data->linear_acceleration.z = 9.81;

  imu_data->angular_velocity.x = 0.0;
  imu_data->angular_velocity.y = 0.0;
  imu_data->angular_velocity.z = 0.0;
  estimator.CallbackImu(imu_data);

  imu_data->header.stamp = ros::Time(101);
  estimator.CallbackImu(imu_data);
  EXPECT_EQ(estimator.getPositionUpdate(), true);

  estimator.AddImuFactor();

  std::shared_ptr<gtsam::NonlinearFactorGraph> current_graph = estimator.getCurrentIncrementalGraph();
  std::shared_ptr<gtsam::Values> current_state = estimator.getCurrentStateGuess();
  int index = estimator.getCurrentIndex();
  EXPECT_EQ(index, 0);
  EXPECT_EQ(estimator.getPositionUpdate(), false);
  EXPECT_EQ(current_graph->size(), 2);
  EXPECT_EQ(current_state->size(), 3);

  //boost::shared_ptr<gtsam::ImuFactor> imu_fac = boost::static_pointer_cast<gtsam::ImuFactor>(current_graph->at(0));

  gtsam::Pose3 x_1 = current_state->at<gtsam::Pose3>(gtsam::symbol_shorthand::X(1));
  gtsam::Vector3 v_1 = current_state->at<gtsam::Vector3>(gtsam::symbol_shorthand::V(1));
  EXPECT_DOUBLE_EQ(x_1.x(), 0.5);
  EXPECT_DOUBLE_EQ(x_1.y(), 0.5);
  EXPECT_DOUBLE_EQ(x_1.z(), 1.0);

  EXPECT_DOUBLE_EQ(x_1.rotation().quaternion().w(), 0.0);
  EXPECT_DOUBLE_EQ(x_1.rotation().quaternion().x(), 1.0);
  EXPECT_DOUBLE_EQ(x_1.rotation().quaternion().y(), 0.0);
  EXPECT_DOUBLE_EQ(x_1.rotation().quaternion().z(), 0.0);

  EXPECT_DOUBLE_EQ(v_1.x(), 1.0);
  EXPECT_DOUBLE_EQ(v_1.y(), 1.0);
  EXPECT_DOUBLE_EQ(v_1.z(), 0.0);
}

TEST(FactorGraphEstimatorLibSimple, TimingCallback) {
  FactorGraphEstimator estimator(tests::test_config_file);

  std::shared_ptr<sensor_msgs::Imu> imu_data = std::make_shared<sensor_msgs::Imu>();
  imu_data->header.stamp = ros::Time(100);

  imu_data->linear_acceleration.x = 1.0;
  imu_data->linear_acceleration.y = 1.0;
  imu_data->linear_acceleration.z = 9.81;

  imu_data->angular_velocity.x = 1.0;
  imu_data->angular_velocity.y = 0.0;
  imu_data->angular_velocity.z = 0.0;
  estimator.CallbackImu(imu_data);

  imu_data->header.stamp = ros::Time(101);
  estimator.CallbackImu(imu_data);

  estimator.TimingCallback(102);

  std::map<int, double> time_map = estimator.getTimeMap();
  EXPECT_EQ(estimator.getCurrentIndex(), 1);
  EXPECT_EQ(time_map.size(), 1);
  EXPECT_DOUBLE_EQ(time_map.at(1), 102);
}


TEST(FactorGraphEstimatorLibSimple, GetStateHistory) {
  FactorGraphEstimator estimator(tests::test_config_file);

  std::shared_ptr<sensor_msgs::Imu> imu_data = std::make_shared<sensor_msgs::Imu>();
  imu_data->header.stamp = ros::Time(100);

  imu_data->linear_acceleration.x = 1.0;
  imu_data->linear_acceleration.y = 1.0;
  imu_data->linear_acceleration.z = 9.81;

  imu_data->angular_velocity.x = 0.0;
  imu_data->angular_velocity.y = 0.0;
  imu_data->angular_velocity.z = 0.0;
  estimator.CallbackImu(imu_data);

  for(int i = 0; i < 50; i++) {
    imu_data->header.stamp += ros::Duration(1.0);
    estimator.CallbackImu(imu_data);
    estimator.TimingCallback(imu_data->header.stamp.toSec());

    std::map<int, double> time_map = estimator.getTimeMap();
    EXPECT_EQ(estimator.getCurrentIndex(), i+1);
    EXPECT_EQ(time_map.size(), i+1);
    EXPECT_EQ(time_map.at(i+1), 101+i);
    estimator.RunOptimize();
  }

  geometry_msgs::PoseArray pose_array = estimator.GetStateHistory();
  EXPECT_EQ(pose_array.poses.size(), 50);
  EXPECT_EQ(pose_array.header.stamp.toSec(), 150);
  for(int i = 0; i < 50; i++) {
    geometry_msgs::Pose pose = pose_array.poses.at(i);

    EXPECT_DOUBLE_EQ(pose.position.x, 0.5*pow(i,2));
    EXPECT_DOUBLE_EQ(pose.position.y, 0.5*pow(i,2));
    EXPECT_DOUBLE_EQ(pose.position.z, 1.0);

    EXPECT_DOUBLE_EQ(pose.orientation.w, 1.0);
    EXPECT_DOUBLE_EQ(pose.orientation.x, 0.0);
    EXPECT_DOUBLE_EQ(pose.orientation.y, 0.0);
    EXPECT_DOUBLE_EQ(pose.orientation.z, 0.0);
  }

}

TEST(FactorGraphEstimatorLibSimple, gpsCallback) {
  FactorGraphEstimator estimator(tests::test_config_file);
  estimator.setImuToGpsPose(gtsam::Pose3());

  std::shared_ptr<sensor_msgs::NavSatFix> msg;
  msg->header.stamp = ros::Time(100);

  msg->latitude = 88;
  msg->longitude = 88;
  msg->altitude = 200;

  estimator.GpsCallback(msg);

  EXPECT_EQ(estimator.getCurrentIndex(), 1);
  EXPECT_EQ(estimator.getCurrentStateGuess()->size(), 4);
  EXPECT_EQ(estimator.getCurrentStateGuess()->exists(gtsam::symbol_shorthand::X(1)), true);
  EXPECT_EQ(estimator.getCurrentStateGuess()->exists(gtsam::symbol_shorthand::G(1)), true);
  EXPECT_EQ(estimator.getCurrentIncrementalGraph()->size(), 5);
  GeographicLib::LocalCartesian enu = estimator.getENUObject();
  EXPECT_DOUBLE_EQ(enu.LatitudeOrigin(), msg->latitude);
  EXPECT_DOUBLE_EQ(enu.LongitudeOrigin(), msg->longitude);
  EXPECT_DOUBLE_EQ(enu.HeightOrigin(), msg->altitude);
}


