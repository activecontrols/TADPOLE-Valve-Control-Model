#ifndef VALVE_CONTROLLER_H
#define VALVE_CONTROLLER_H

/*
 * valve_controller.hpp
 *
 *  Created on: 2024-10-11 by Robert Nies
 *  Description: Code for open loop valve control
 */

struct Venturi {
  double inlet_area;  // in^2
  double throat_area; // in^2
  double cd;
};

struct Fluid_Line {
  float valve_upstream_pressure;       // psi
  float valve_downstream_pressure;     // psi
  float venturi_differential_pressure; // psi
  float venturi_temperature;           // K
  float valve_temperature;             // K
};

struct Sensor_Data {
  double chamber_pressure; // psi
  Fluid_Line ox;
  Fluid_Line ipa;
};

void open_loop_thrust_control(double thrust, double ox_valve_upstream, double ipa_valve_upstream, double *angle_ox, double *angle_ipa);
void closed_loop_thrust_control(double thrust, double time_delta, double mfr_ox, double mfr_ipa, double chamber_pressure_sensor,
                                double ox_valve_upstream, double ipa_valve_upstream, double *cp_err_sum, double *ox_err_sum, double *ipa_err_sum,
                                double *angle_ox, double *angle_ipa);
#endif