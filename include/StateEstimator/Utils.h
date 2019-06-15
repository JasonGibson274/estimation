#ifndef UTILS_MSGS_H_
#define UTILS_MSGS_H_
namespace alphapilot {
struct IMU_readings
{
  double x_accel = 0.0;
  double y_accel = 0.0;
  double z_accel = 0.0;
  double roll_vel = 0.0;
  double pitch_vel = 0.0;
  double yaw_vel = 0.0;
  double dt = 0.0;
};

struct drone_covariance {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  double x_dot = 0.0;
  double y_dot = 0.0;
  double z_dot = 0.0;
  double roll_dot = 0.0;
  double pitch_dot = 0.0;
  double yaw_dot = 0.0;
};

struct drone_state
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double qx = 0.0;
  double qy = 0.0;
  double qz = 0.0;
  double qw = 0.0;
  double x_dot = 0.0;
  double y_dot = 0.0;
  double z_dot = 0.0;
  double roll_dot = 0.0;
  double pitch_dot = 0.0;
  double yaw_dot = 0.0;
  // seconds
  double time = 0.0;
  drone_covariance covariance;
};

struct camera_info {
 double fx = 0.0;
 double fy = 0.0;
 double s = 0.0;
 double u0 = 0.0;
 double v0 = 0.0;
};
} // StateEstimator
#endif // UTILS_MSGS_H_
