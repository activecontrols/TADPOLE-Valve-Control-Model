#ifndef PI_CONTROLLER_H
#define PI_CONTROLLER_H

class PI_Controller {
public:
  PI_Controller(double kp, double ki, double *err_sum);
  double compute(double input_error, double time_delta);

private:
  double kp;
  double ki;
  double *err_sum = 0;
};

namespace ClosedLoopControllers {
extern PI_Controller Chamber_Pressure_Controller;
extern PI_Controller LOX_Angle_Controller;
extern PI_Controller IPA_Angle_Controller;
} // namespace ClosedLoopControllers

#endif