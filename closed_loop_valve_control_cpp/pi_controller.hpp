#ifndef PI_CONTROLLER_H
#define PI_CONTROLLER_H

class PI_Controller {
public:
  PI_Controller(double kp, double ki);
  double compute(double input_error, double time_delta, double *err_sum);

private:
  double kp;
  double ki;
};

namespace ClosedLoopControllers {
extern PI_Controller Chamber_Pressure_Controller;
extern PI_Controller LOX_Angle_Controller;
extern PI_Controller IPA_Angle_Controller;
} // namespace ClosedLoopControllers

#endif