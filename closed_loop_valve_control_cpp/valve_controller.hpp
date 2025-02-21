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
  double tank_pressure;             // psi
  double venturi_upstream_pressure; // psi
  double venturi_throat_pressure;   // psi
  double venturi_temperature;       // K
  double valve_temperature;         // K
};

struct Sensor_Data {
  double chamber_pressure; // psi
  Fluid_Line ox;
  Fluid_Line ipa;
};

void open_loop_thrust_control(double thrust, Sensor_Data sensor_data, double *angle_ox, double *angle_ipa);
void open_loop_thrust_control_defaults(double thrust, double *angle_ox, double *angle_ipa);
void closed_loop_thrust_control(double thrust, double time_delta, double mfr_ox, double mfr_ipa, double chamber_pressure_sensor,
                                double *cp_err_sum, double *ox_err_sum, double *ipa_err_sum, double *angle_ox, double *angle_ipa);
#endif