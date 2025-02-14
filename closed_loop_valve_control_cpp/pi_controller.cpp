#include "pi_controller.hpp"

PI_Controller::PI_Controller(double kp, double ki, double *err_sum) {
  this->kp = kp;
  this->ki = ki;
  this->err_sum = err_sum;
}

double PI_Controller::compute(double input_error, double time_delta) {
  *this->err_sum += input_error * time_delta;
  return this->kp * input_error + this->ki * *this->err_sum;
}

namespace ClosedLoopControllers {
static double err_sum_cp;
static double err_sum_lox;
static double err_sum_ipa;
PI_Controller Chamber_Pressure_Controller(0, 1, &err_sum_cp); // TODO RJN CL - PID constants
PI_Controller LOX_Angle_Controller(0, 1, &err_sum_lox);
PI_Controller IPA_Angle_Controller(0, 1, &err_sum_ipa);
} // namespace ClosedLoopControllers
