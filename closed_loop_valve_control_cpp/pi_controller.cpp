#include "pi_controller.hpp"

PI_Controller::PI_Controller(double kp, double ki) {
  this->kp = kp;
  this->ki = ki;
}

double PI_Controller::compute(double input_error, double time_delta, double *err_sum) {
  *err_sum += input_error * time_delta;
  return this->kp * input_error + this->ki * *err_sum;
}

namespace ClosedLoopControllers {
PI_Controller Chamber_Pressure_Controller(6e-3 * 0, 6e-2);
PI_Controller LOX_Angle_Controller(35, 65);
PI_Controller IPA_Angle_Controller(35, 65);
} // namespace ClosedLoopControllers
