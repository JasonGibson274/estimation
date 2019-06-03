//
// Created by jason on 6/1/19.
//

#include <StateEstimator/estimation/FactorGraph/FactorGraphEstimator.h>

using namespace alphapilot::estimator;
using namespace alphapilot;

int main() {
  std::shared_ptr<drone_state> init_state = std::make_shared<drone_state>();
  init_state->z = 1.0;
  FactorGraphEstimator estimator(init_state);
  std::cout << "\ninit ended\n" << std::endl;
  std::cout << "\nstarting imu callback\n" << std::endl;
  std::shared_ptr<IMU_readings> reading = std::make_shared<IMU_readings>();
  reading->x_accel = -0.21;
  reading->y_accel = -0.77;
  reading->z_accel = -9.22;
  reading->roll_vel = -0.04;
  reading->pitch_vel = -0.02;
  reading->yaw_vel = -0.05;
  reading->dt = 0.1;
  estimator.callback_imu(reading);
  std::cout << "\nimu callback ended\n" << std::endl;
  std::cout << "\nstarting camera callback\n" << std::endl;
  std::shared_ptr<std::map<std::string, std::pair<double, double>>> camera_reading = std::make_shared<std::map<std::string, std::pair<double, double>>>();
  camera_reading->insert(std::make_pair("Gate10-1", std::make_pair(402.281, 195.785)));
  camera_reading->insert(std::make_pair("Gate12-3", std::make_pair(55.6591, 147.801)));
  estimator.callback_cm(camera_reading);
  estimator.latest_state();

  std::cout << "\n\nfinished code\n\n" << std::endl;
  return 0;
}
