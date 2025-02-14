#include "pi_controller.hpp"

PI_Controller::PI_Controller(double kp, double ki) {
  this->kp = kp;
  this->ki = ki;
}

double PI_Controller::compute(double input_error, double time_delta, double *err_sum) {
  *err_sum += input_error * time_delta;
  // if (*err_sum > 5) {
  //   *err_sum = 5;
  // }
  // if (*err_sum < -5) {
  //   *err_sum = -5;
  // }
  return this->kp * input_error + this->ki * *err_sum;
}

namespace ClosedLoopControllers {
PI_Controller Chamber_Pressure_Controller(0.05, 0.2); // TODO RJN CL - PID constants
PI_Controller LOX_Angle_Controller(2, 4);
PI_Controller IPA_Angle_Controller(0, 0);
} // namespace ClosedLoopControllers
