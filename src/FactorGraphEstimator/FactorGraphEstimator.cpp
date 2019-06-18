#include <StateEstimator/estimation/FactorGraph/FactorGraphEstimator.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>

using namespace std;
using namespace gtsam;

const double kGravity = 9.81;

namespace alphapilot {
namespace estimator {
  /**
   * Constructor for the Factor Graph Estimator; sets up the noise models
   * @param initial_state, start location to begin the optimization
   */
  FactorGraphEstimator::FactorGraphEstimator(const std::shared_ptr<drone_state>& initial_state, bool debug) {
    debug_ = debug;

    // init the set of Values passed to GTSAM during optimization
    vel_change_accum_ = Vector3(0,0,0);

    // init camera to flightgoggles default, TODO have correct estimate
    K_ = boost::make_shared<gtsam::Cal3_S2>(548.4088134765625, 548.4088134765625, 0, 512.0, 384.0);
    camera_transform_ = Pose3(Rot3::Quaternion(-0.5, 0.5, -0.5, 0.5), Point3(0,0,0));

    // setup IMU preintegrator parameters
    boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> p = PreintegratedCombinedMeasurements::Params::MakeSharedU(GRAVITY);
    double accel_noise_sigma = 2.0;
    double gyro_noise_sigma = 0.1;
    double accel_bias_rw_sigma = 0.1;
    double gyro_bias_rw_sigma = 0.1;
    Matrix33 measured_acc_cov = Matrix33::Identity(3,3) * pow(accel_noise_sigma,2);
    Matrix33 measured_omega_cov = Matrix33::Identity(3,3) * pow(gyro_noise_sigma,2);
    Matrix33 integration_error_cov = Matrix33::Identity(3,3)*1e-4; // error committed in integrating position from velocities
    Matrix33 bias_acc_cov = Matrix33::Identity(3,3) * pow(accel_bias_rw_sigma,2);
    Matrix33 bias_omega_cov = Matrix33::Identity(3,3) * pow(gyro_bias_rw_sigma,2);
    Matrix66 bias_acc_omega_int = Matrix::Identity(6,6)*0.1; // error in the bias used for preintegration

    // add them to the params
    p->accelerometerCovariance = measured_acc_cov; // acc white noise in continuous
    p->integrationCovariance = integration_error_cov; // integration uncertainty continuous
    p->gyroscopeCovariance = measured_omega_cov; // gyro white noise in continuous
    p->biasAccCovariance = bias_acc_cov; // acc bias in continuous
    p->biasOmegaCovariance = bias_omega_cov; // gyro bias in continuous
    p->biasAccOmegaInt = bias_acc_omega_int;

    // use regular preintegrated
    preintegrator_imu_ = PreintegratedImuMeasurements(p, gtsam::imuBias::ConstantBias());

    // Construct parameters for ISAM optimizer
    ISAM2Params isam_parameters;
    isam_parameters.relinearizeThreshold = 0.01;
    isam_parameters.relinearizeSkip = 1;
    isam_parameters.cacheLinearizedFactors = false;
    isam_parameters.enableDetailedResults = true;
    isam_parameters.print();
    isam_ = ISAM2(isam_parameters);

    // Initialize Noise Models
    cam_measurement_noise_ =
      noiseModel::Isotropic::Sigma(2, 10.0);  // one pixel in u and v

    Vector6 covvec;
    covvec << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
    std::cout << "bias noise = " << covvec << std::endl;
    bias_noise_ = noiseModel::Diagonal::Variances(covvec);

    // pose factor noise
    Vector3 odom_vel_noise = Vector3(0.1,0.1,0.1);
    std::cout << "odom vel noise = " << odom_vel_noise << std::endl;
    odometry_vel_noise_ = noiseModel::Diagonal::Sigmas(odom_vel_noise);
    Vector6 pose_noise;
    pose_noise << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
    std::cout << "pose noise = " << pose_noise << std::endl;
    odometry_pose_noise_ = noiseModel::Diagonal::Sigmas(pose_noise);

    // add the priors on state
    add_priors(initial_state);
  }

  /**
   * callback for the imu, integrates the reading using the preintegrator_imu_, and then
   * calls propagate_imu to update the estimated state given to GTSAM
   * @param imu_data, assumes the accel and angular vel was over the last dt seconds
   */
  void FactorGraphEstimator::callback_imu(const std::shared_ptr<IMU_readings> imu_data) {
    if(!use_imu_factors_) {
      return;
    }
    position_update_ = true;
    // -1 if invert, 1 if !invert
    double invert_x = invert_x_ ? -1.0 : 1.0;
    double invert_y = invert_y_ ? -1.0 : 1.0;
    double invert_z = invert_z_ ? -1.0 : 1.0;

    // updates the current guesses of position
    Vector3 accel = Vector3 (imu_data->x_accel * invert_x, imu_data->y_accel * invert_y, imu_data->z_accel * invert_z);
    Vector3 ang_rate = Vector3 (imu_data->roll_vel * invert_x, imu_data->pitch_vel * invert_y, imu_data->yaw_vel * invert_z);
    lock_guard<mutex> preintegration_lck(preintegrator_lck_);
    preintegrator_imu_.integrateMeasurement(accel, ang_rate, imu_data->dt);
    imu_meas_count_++;

    // integrate the IMU to get an updated estimate of the current position
    // integrate the imu reading using affine dynamics from TODO cite
    propagate_imu(accel, ang_rate, imu_data->dt);
  }

  /**
   * Creates a new state variable by increasing the state index_
   * and adds it to the graph.
   * The basic process for doing so is creating a factor relating
   * previous state variables to current state variables (aka. motion model)
   * and then adding a guess for the current values of those states
   * to give the optimizer a value to start from
   */
  void FactorGraphEstimator::add_imu_factor() {
    if(imu_meas_count_ <= 0) {
      return;
    }

    // Update bias at a slower rate
    if (index_ % imu_bias_incr_ == 0) {
      bias_index_++;
      Symbol b1 = symbol_shorthand::B(bias_index_ - 1);
      Symbol b2 = symbol_shorthand::B(bias_index_);

      // Motion model of bias - currently constant bias is assumed

      // Add factor to graph and add initial variable guesses
      lock_guard<mutex> lck(graph_lck_);
      current_incremental_graph_.emplace_shared<BetweenFactor<
          imuBias::ConstantBias>>(b1, b2, imuBias::ConstantBias(),
                                  bias_noise_);
      gtsam_current_state_initial_guess_.insert(symbol_shorthand::B(bias_index_),
                                                 current_bias_guess_);
    }

    preintegrator_lck_.lock();
    // motion model of position and velocity variables
    ImuFactor imufac(symbol_shorthand::X(index_ - 1),
                     symbol_shorthand::V(index_ - 1),
                     symbol_shorthand::X(index_),
                     symbol_shorthand::V(index_),
                     symbol_shorthand::B(bias_index_),
                     preintegrator_imu_);

    // clear IMU rolling integration
    preintegrator_imu_.resetIntegration();
    imu_meas_count_ = 0;

    // Add factor to graph and add initial variable guesses
    lock_guard<mutex> lck(graph_lck_);
    gtsam_current_state_initial_guess_.insert(symbol_shorthand::X(index_),
                                              current_position_guess_);
    gtsam_current_state_initial_guess_.insert(symbol_shorthand::V(index_),
                                              current_velocity_guess_);
    // prevents current_vel and pos from being updated
    preintegrator_lck_.unlock();
    current_incremental_graph_.add(imufac);
  }

  /**
   * Takes in detected landmarks from image space and adds them to the factor
   *  graph using the structureless landmark factor from the following paper:
   *
   * C. Forster, L. Carlone, F. Dellaert, D. Scaramuzza, "IMU Preintegration on
   * Manifold for Efficient Visual-Inertial Maximum-a-Posteriori Estimation",
   * Robotics: Science and Systems (RSS), 2015.
   *
   * Optimizes the enitre state trajecotry at the end
   *
   * @param map<landmark_id, uv_coords> landmark_data
   */
  void FactorGraphEstimator::callback_cm(const std::shared_ptr<map<std::string, pair<double, double>>>
                                         landmark_data) {

    // TODO verify that the landmark data is in the camera FOV

    // if there is no position updates this is unconstrained
    if(!position_update_) {
      std::cerr << "ERROR: no position updates have happened but camera was recieved\n" <<
                   "Not optimizing otherwise unconstrained optimization" << std::endl;
      return;
    }
    // Create newest state
    add_factors();

    if(!use_camera_factors_) {
      return;
    }
    for (const auto& seen_landmark : *landmark_data) {
      //std::cout << "adding landmark detection " << seen_landmark.first << ", " << seen_landmark.second.first << ", " << seen_landmark.second.second << std::endl;
      std::string l_id = seen_landmark.first;
      pair<double, double> im_coords = seen_landmark.second;
      auto landmark_factor_it = landmark_factors_.find(l_id);

      // New Landmark - Add a new Factor
      if (landmark_factor_it == landmark_factors_.end()) {
        SmartProjectionPoseFactor<Cal3_S2>::shared_ptr smartFactor(new SmartProjectionPoseFactor<Cal3_S2>(cam_measurement_noise_, K_, camera_transform_));
        landmark_factors_[l_id] = smartFactor;
        current_incremental_graph_.push_back(smartFactor);
      }
      // Translate detection into gtsam
      Point2 detection_coords(im_coords.first, im_coords.second);

      // Add landmark to factor
      graph_lck_.lock();
      landmark_factors_[l_id]->add(detection_coords,
                                   symbol_shorthand::X(index_));
      graph_lck_.unlock();
    }
  }

  std::vector<std::array<double, 3>> FactorGraphEstimator::get_landmark_positions() {
    // TODO return variable, set up elsewhere when optimize is called
    std::vector<std::array<double, 3>> result;
    //for(auto it = landmark_factors_.begin(); it != landmark_factors_.end(); it++) {
    //  boost::optional<Point3> point = it->point();
    //}
    return result;
  }



  /**
   * Adds the local factor graph that was constructed during the most recent
   * callbacks, then clears the graph for the next iteration.
   * This optimizes the entire state trajectory
   */
  void FactorGraphEstimator::run_optimize() {
    if(current_incremental_graph_.empty()) {
      std::cerr << "\n\nERROR: cannot optimize over a empty graph\n" << std::endl;
      return;
    }
    position_update_ = false;
    // run the optimization and output the final state
    lock_guard<mutex> graph_lck(graph_lck_);
    lock_guard<mutex> preintegration_lck(preintegrator_lck_);


    //NonlinearFactorGraph isam_graph = current_incremental_graph_.clone();
    if(debug_) {
      std::cout << "\n\n incremental graph" << std::endl;
      current_incremental_graph_.print();
      std::cout << "\n\n guess state guesses" << std::endl;
      gtsam_current_state_initial_guess_.print();
    }

    // run update and run optimization
    isam_.update(current_incremental_graph_, gtsam_current_state_initial_guess_);

    // set the guesses of state to the correct output
    current_position_guess_ = isam_.calculateEstimate<Pose3>(symbol_shorthand::X(index_));
    current_velocity_guess_ = isam_.calculateEstimate<Vector3>(symbol_shorthand::V(index_));
    if(use_imu_factors_) {
      current_bias_guess_ = isam_.calculateEstimate<imuBias::ConstantBias>(symbol_shorthand::B(bias_index_));
      // print bias
      std::cout << "bias = " << current_bias_guess_ << std::endl;
    }

    if(debug_) {
      std::cout << "\n\n current position guess" << std::endl;
      current_position_guess_.print();
      std::cout << "\n\n current velocity guess" << std::endl;
      std::cout << current_velocity_guess_ << std::endl;
    }

    // set the result to best guess
    current_pose_estimate_.x = current_position_guess_.x();
    current_pose_estimate_.y = current_position_guess_.y();
    current_pose_estimate_.z = current_position_guess_.z();

    current_pose_estimate_.qw = current_position_guess_.rotation().quaternion()[0];
    current_pose_estimate_.qx = current_position_guess_.rotation().quaternion()[1];
    current_pose_estimate_.qy = current_position_guess_.rotation().quaternion()[2];
    current_pose_estimate_.qz = current_position_guess_.rotation().quaternion()[3];

    current_pose_estimate_.x_dot = current_velocity_guess_[0];
    current_pose_estimate_.y_dot = current_velocity_guess_[1];
    current_pose_estimate_.z_dot = current_velocity_guess_[2];

    // clear incremental graph and current guesses
    current_incremental_graph_ = NonlinearFactorGraph();
    gtsam_current_state_initial_guess_.clear();

  }

  /**
   * Propagates the state according to the (stochastic) rigid body dynamics equations
   * Modifies class members
   * @param acc linear acceleration in body frame
   * @param angular_vel in body frame
   * @param dt time difference
   */
   // TODO clean up method
  void FactorGraphEstimator::propagate_imu(Vector3 acc, Vector3 angular_vel, double dt) {
    //TODO fix IMU motion model to match what is happening in real life
    drone_state result;

    double x = current_position_guess_.x();
    double y = current_position_guess_.y();
    double z = current_position_guess_.z();

    double x_dot = current_velocity_guess_[0];
    double y_dot = current_velocity_guess_[1];
    double z_dot = current_velocity_guess_[2];

    double roll = current_position_guess_.rotation().rpy()[0];
    double pitch = current_position_guess_.rotation().rpy()[1];
    double yaw = current_position_guess_.rotation().rpy()[2];

    double roll_dot = angular_vel[0];
    double pitch_dot = angular_vel[1];
    double yaw_dot = angular_vel[2];

    if(debug_) {
      std::cout.precision(10);
      std::cout << "===== prop =====" << std::endl;
      std::cout << "dt = " << dt << std::endl;
      std::cout << "ang rate before = " << roll_dot << ", " << pitch_dot << ", " << yaw_dot << std::endl;
      std::cout << "acc beore = " << acc[0] << ", " << acc[1] << ", " << acc[2] << std::endl;
      std::cout << "=====" << std::endl;

      std::cout << "pos before = " << x << ", " << y << ", " << z << std::endl;
      std::cout << "rpy before = " << roll << ", " << pitch << ", " << yaw << std::endl;
      std::cout << "vel before = " << x_dot << ", " << y_dot << ", " << z_dot << std::endl;
    }

    //Update angular rate
    result.roll_dot = roll_dot;
    result.pitch_dot = pitch_dot;
    result.yaw_dot = yaw_dot;

    //Position update
    result.x = x + x_dot*dt + 0.5*acc[0]*pow(dt, 2);
    result.y = y + y_dot*dt + 0.5*acc[1]*pow(dt, 2);
    result.z = z + z_dot*dt;// + 0.5*acc[2]*pow(dt, 2);

    //Update velocity
    float c_phi, c_theta, c_psi, s_phi, s_theta, s_psi;
    float ux, uy, uz;
    c_phi = cosf(roll);
    c_theta = cosf(pitch);
    c_psi = cosf(yaw);
    s_phi = sinf(roll);
    s_theta = sinf(pitch);
    s_psi = sinf(yaw);
    ux = acc[0]*dt;
    uy = acc[1]*dt;
    uz = acc[2]*dt;
    result.x_dot = x_dot + (c_theta*c_psi)*ux + (s_phi*s_theta*c_psi - c_phi*s_psi)*uy + (c_phi*s_theta*c_psi + s_phi*s_psi)*uz;
    result.y_dot = y_dot + (c_theta*s_psi)*ux + (s_phi*s_theta*s_psi + c_phi*c_psi)*uy + (c_phi*s_theta*s_psi - s_phi*c_psi)*uz;
    result.z_dot = z_dot + (-s_theta)*ux + (c_theta*s_phi)*uy + (c_theta*c_phi)*uz - GRAVITY*dt;

    //Update the euler angles
    float r_result, p_result, y_result;
    r_result = roll + (roll_dot + (s_phi*s_theta/c_theta)*pitch_dot + (c_phi*s_theta/c_theta)*yaw_dot)*dt;
    p_result = pitch + (c_phi*pitch_dot - s_phi*yaw_dot)*dt;
    y_result = yaw + (s_phi/c_theta*pitch_dot + c_phi/c_theta*yaw_dot)*dt;

    // apply the update
    // the ordering is correct for gtsam
    //std::cout << "\n\n vel before: \n" << *current_velocity_guess_ << std::endl;

    current_position_guess_ = Pose3(Rot3::Ypr(y_result, p_result, r_result),
            Point3(result.x, result.y, result.z));
    current_velocity_guess_ = Vector3(result.x_dot, result.y_dot, result.z_dot);
    //std::cout << "\n\n vel after: \n" << *current_velocity_guess_ << std::endl;
    if(debug_) {
      std::cout << "===== after ====" << std::endl;
      std::cout << "pos after = " << result.x << ", " << result.y << ", " << result.z << std::endl;
      std::cout << "rpy after = " << r_result << ", " << p_result << ", " << y_result << std::endl;
      std::cout << "vel after = " << result.x_dot << ", " << result.y_dot << ", " << result.z_dot << std::endl;
    }
  }

  void FactorGraphEstimator::callback_range(int rangestuff) {
    if(!use_range_factors_) {
      return;
    }

  }

  void FactorGraphEstimator::add_priors(const std::shared_ptr<drone_state> initial_state) {
    current_position_guess_ = Pose3(Rot3::Quaternion(initial_state->qw, initial_state->qx, initial_state->qy, initial_state->qz),
            Point3(initial_state->x,initial_state->y,initial_state->z));
    current_position_guess_.print();
    current_velocity_guess_ = Vector3(initial_state->x_dot, initial_state->y_dot, initial_state->z_dot);
    Vector6 bias_tmp;
    bias_tmp << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    std::cout << "bias temp = " << current_bias_guess_ << std::endl;
    current_bias_guess_ = imuBias::ConstantBias(bias_tmp);
    gtsam_current_state_initial_guess_.clear();

    // insert initial guesses of state
    gtsam_current_state_initial_guess_.insert(symbol_shorthand::X(index_),
                                              current_position_guess_);
    gtsam_current_state_initial_guess_.insert(symbol_shorthand::V(index_),
                                              current_velocity_guess_);
    if(use_imu_factors_) {
      gtsam_current_state_initial_guess_.insert(symbol_shorthand::B(bias_index_),
                                              current_bias_guess_);
    }


    //gtsam_current_state_initial_guess_->print();

    // create priors on the state

    // Assemble prior noise model and add it the graph.
    noiseModel::Diagonal::shared_ptr pose_noise_model = noiseModel::Diagonal::Sigmas((Vector(6)
            << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1).finished()); // rad,rad,rad,m, m, m
    std::cout << "\npose noise prior pointer = " << std::endl;
    std::cout << pose_noise_model->sigmas();
    noiseModel::Diagonal::shared_ptr velocity_noise_model = noiseModel::Isotropic::Sigma(3,0.1); // m/s
    std::cout << "\nvel prior pointer = " << std::endl;
    std::cout << velocity_noise_model->sigmas();
    noiseModel::Diagonal::shared_ptr bias_noise_model = noiseModel::Isotropic::Sigma(6,1e-3);
    std::cout << "\nbias_noise_model prior = " << std::endl;
    std::cout << bias_noise_model->sigmas();

    // priors match initial guess
    current_incremental_graph_.add(PriorFactor<Pose3>(symbol_shorthand::X(index_), current_position_guess_, pose_noise_model));
    current_incremental_graph_.add(PriorFactor<Vector3>(symbol_shorthand::V(index_), current_velocity_guess_, velocity_noise_model));
    if(use_imu_factors_) {
      current_incremental_graph_.add(PriorFactor<imuBias::ConstantBias>(symbol_shorthand::B(bias_index_), current_bias_guess_, bias_noise_model));
    }

    // set the result to initial guess
    current_pose_estimate_.x = current_position_guess_.x();
    current_pose_estimate_.y = current_position_guess_.y();
    current_pose_estimate_.z = current_position_guess_.z();

    current_pose_estimate_.qw = current_position_guess_.rotation().quaternion()[0];
    current_pose_estimate_.qx = current_position_guess_.rotation().quaternion()[1];
    current_pose_estimate_.qy = current_position_guess_.rotation().quaternion()[2];
    current_pose_estimate_.qz = current_position_guess_.rotation().quaternion()[3];

    current_pose_estimate_.x_dot = current_velocity_guess_[0];
    current_pose_estimate_.y_dot = current_velocity_guess_[1];
    current_pose_estimate_.z_dot = current_velocity_guess_[2];

    last_pose_state_ = current_pose_estimate_;

  }

  void FactorGraphEstimator::resetGraph(const std::shared_ptr<drone_state> initial_state) {
    lock_guard<mutex> graph_lck(graph_lck_);
    lock_guard<mutex> preintegration_lck(preintegrator_lck_);
    lock_guard<mutex> pose_lck(pose_lck_);

    //clear
    index_ = 0;
    bias_index_ = 0;
    pose_message_count_ = 0;
    imu_meas_count_ = 0;
    current_incremental_graph_ = NonlinearFactorGraph();
    preintegrator_imu_.resetIntegration();
    ISAM2Params isam_parameters;
    isam_parameters.relinearizeThreshold = 0.01;
    isam_parameters.relinearizeSkip = 1;
    isam_parameters.cacheLinearizedFactors = false;
    isam_parameters.enableDetailedResults = true;
    isam_parameters.print();
    isam_ = ISAM2(isam_parameters);

    // reset up priors
    add_priors(initial_state);
  }

  drone_state FactorGraphEstimator::latest_state() {
    if(position_update_) {
      run_optimize();
    }
    lock_guard<mutex> graph_lck(graph_lck_);
    return current_pose_estimate_;
  }

  /**
   * Takes in a global estimate of pose, finds the difference and accumulates the delta for the factor
   * @param odom_data
   */
  void FactorGraphEstimator::callback_odometry(const std::shared_ptr<drone_state> odom_data) {
    if(!use_pose_factors_) {
      return;
    }
    // otherwise you can have a between factor of two states that do not exist
    if(!use_imu_factors_) {
      position_update_ = true;
    }
    lock_guard<mutex> pose_lck(pose_lck_);

    pose_message_count_++;

    Quaternion odom_q = Quaternion(odom_data->qw, odom_data->qx, odom_data->qy, odom_data->qz);
    odom_q.normalize();
    Quaternion last_pose_q(last_pose_state_.qw, last_pose_state_.qx, last_pose_state_.qy, last_pose_state_.qz);
    last_pose_q.normalize();

    // find the difference between the two quaternions
    Rot3 rot_update = traits<Quaternion>::Between(last_pose_q, odom_q);
    pose_rot_accum_ = pose_rot_accum_ * rot_update;

    Point3 trans_update = Point3(odom_data->x - last_pose_state_.x, odom_data->y - last_pose_state_.y,
        odom_data->z - last_pose_state_.z);
    pose_trans_accum_ += trans_update;

    Vector3 vel_update (odom_data->x_dot - last_pose_state_.x_dot, odom_data->y_dot - last_pose_state_.y_dot,
        odom_data->z_dot - last_pose_state_.z_dot);
    //std::cout << "pose diff " << pos_update << std::endl;
    noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.2));

    vel_change_accum_ += vel_update;

    last_pose_state_ = *odom_data;

  }

  void FactorGraphEstimator::add_pose_factor() {
    if(pose_message_count_ <= 0) {
      return;
    }
    lock_guard<mutex> graph_lck(graph_lck_);
    lock_guard<mutex> pose_lck(pose_lck_);
    pose_message_count_ = 0;

    // set up the state guesses from odometry if imu is turned off
    if(!use_imu_factors_) {
      // set up guesses before transforming to body frame for between factor
      Rot3 new_rot = pose_rot_accum_ * current_position_guess_.rotation();

      Point3 new_point = current_position_guess_.translation() + pose_trans_accum_;
      gtsam_current_state_initial_guess_.insert(symbol_shorthand::X(index_),
                                              Pose3(new_rot, new_point));

      Vector3 temp_vel = current_velocity_guess_ + vel_change_accum_;
      gtsam_current_state_initial_guess_.insert(symbol_shorthand::V(index_), temp_vel);
    }



    // add constraint on the poses
    // transforms the position offset into body frame offset
    Point3 body_trans = current_position_guess_.transformTo(pose_trans_accum_ + current_position_guess_.translation());

    // assumes that the pose change is in body frame
    current_incremental_graph_.emplace_shared<BetweenFactor<Pose3>>(symbol_shorthand::X(index_-1),
        symbol_shorthand::X(index_), Pose3(pose_rot_accum_, body_trans), odometry_pose_noise_);

    // add constraint on the velocity
    current_incremental_graph_.emplace_shared<BetweenFactor<Vector3>>(symbol_shorthand::V(index_-1),
        symbol_shorthand::V(index_), vel_change_accum_, odometry_vel_noise_);

    vel_change_accum_ = Vector3(0,0,0);
    pose_rot_accum_ = Rot3();
    pose_trans_accum_ = Point3(0,0,0);
  }

  void FactorGraphEstimator::add_factors() {
    if(!position_update_) {
      return;
    }
    index_++;
    add_pose_factor();
    add_imu_factor();
  }


} // estimator
} // StateEstimator
