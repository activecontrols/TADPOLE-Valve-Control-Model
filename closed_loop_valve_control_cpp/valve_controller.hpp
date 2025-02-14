#ifndef VALVE_CONTROLLER_H
#define VALVE_CONTROLLER_H

/*
 * valve_controller.hpp
 *
 *  Created on: 2024-10-11 by Robert Nies
 *  Description: Code for open loop valve control
 */

struct cav_vent {
  double cav_vent_throat_area; // in^2
  double cav_vent_cd;          // unitless
};

struct fluid {
  double vapour_pressure; // psi
  double density;         // lb / in^3
};

struct sensor_data {
  double chamber_pressure;      // psi
  double ox_tank_pressure;      // psi
  double ipa_tank_pressure;     // psi
  double ox_valve_temperature;  // K
  double ipa_valve_temperature; // K
  double ox_cv_temperature;     // K
  double ipa_cv_temperature;    // K
};

void closed_loop_thrust_control(double thrust, double time_delta, double mfr_ox, double mfr_ipa, double *angle_ox, double *angle_fuel);
#endif