#ifndef UTILS_MSGS_H_
#define UTILS_MSGS_H_

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
#endif // UTILS_MSGS_H_
