#ifndef UTILS_MSGS_H_
#define UTILS_MSGS_H_
namespace alphapilot {
struct IMU_readings
{
  double x_accel = 0;
  double y_accel = 0;
  double z_accel = 0
  double roll_vel = 0;
  double pitch_vel = 0;
  double yaw_vel = 0;
  double dt = 0;
};

struct drone_state
{
  double x = 0;
  double y = 0;
  double z = 0;
  double roll = 0;
  double pitch = 0;
  double yaw = 0;
  double x_dot = 0;
  double y_dot = 0;
  double z_dot = 0;
  double roll_dot = 0;
  double pitch_dot = 0;
  double yaw_dot = 0;
};
} // alphapilot
#endif // UTILS_MSGS_H_
