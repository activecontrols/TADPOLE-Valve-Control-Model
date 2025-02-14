#include "mex.h"
#include "valve_controller.hpp"

// MEX gateway function
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  // Check number of inputs
  if (nrhs != 4) {
    mexErrMsgIdAndTxt("MATLAB:myFunction:nrhs", "One input required.");
  }

  // Check number of outputs
  if (nlhs != 2) {
    mexErrMsgIdAndTxt("MATLAB:myFunction:nlhs", "Two outputs required.");
  }

  double *thrust = mxGetPr(prhs[0]);
  double *time_delta = mxGetPr(prhs[1]);
  double *mfr_ox = mxGetPr(prhs[2]);
  double *mfr_ipa = mxGetPr(prhs[3]);

  // Create output variables
  plhs[0] = mxCreateDoubleScalar(0); // First output (y1)
  plhs[1] = mxCreateDoubleScalar(0); // Second output (y2)

  // Get pointers to output variables
  double *ox_angle = mxGetPr(plhs[0]);
  double *ipa_angle = mxGetPr(plhs[1]);

  closed_loop_thrust_control(*thrust, *time_delta, *mfr_ox, *mfr_ipa, ox_angle, ipa_angle);
}