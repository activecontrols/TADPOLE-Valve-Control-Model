#include "valve_controller.hpp"
#include "pi_controller.hpp"
#include <math.h>

// #include <Router.h>
// #include "CString.h"

#define tadpole_AREA_OF_THROAT 1.69 // in^2
#define tadpole_C_STAR 4998.0654    // ft / s // TODO RJN OL - replace with data from testing
#define tadpole_MASS_FLOW_RATIO 1.2 // #ox = 1.2 * ipa
#define GRAVITY_FT_S 32.1740        // Gravity in (ft / s^2)

#define IN3_TO_GAL 0.004329       // convert cubic inches to gallons
#define PER_SEC_TO_PER_MIN 60     // convert per second to per minute
#define LB_TO_TON 0.000453592     // convert lb to metric tons
#define PER_IN3_TO_PER_M3 61023.7 // convert per in^3 to per m^3

#define INTERPOLATION_TABLE_LENGTH 30 // max length of all tables - set to enable passing tables to functions
#define VALVE_ANGLE_TABLE_LEN 11
// CV (assume unitless) to angle (degrees)
double valve_angle_table[2][INTERPOLATION_TABLE_LENGTH] = {
    {0.000, 0.070, 0.161, 0.378, 0.670, 1.000, 1.450, 2.050, 2.780, 3.710, 4.960},
    {0, 9, 18, 27, 36, 45, 54, 63, 72, 81, 90}};

#define CF_THRUST_TABLE_LEN 2 // TODO RJN OL - replace with data from testing
// thrust (lbf) to cf (unitless)
double cf_thrust_table[2][INTERPOLATION_TABLE_LENGTH] = {
    {220, 550},
    {1.12, 1.3}};

#define OX_DENSITY_TABLE_LEN 20
// temperature (K) to density (lb/in^3)
double ox_density_table[2][INTERPOLATION_TABLE_LENGTH] = {
    {55, 60, 65, 70, 75, 80, 85, 90, 95, 100, 105, 110, 115, 120, 125, 130, 135, 140, 145, 150},
    {0.04709027778, 0.04631539352, 0.04550925926, 0.0446880787, 0.04385474537, 0.04300810185, 0.04214525463, 0.04126099537, 0.04035127315, 0.03941087963, 0.03843229167, 0.03740856481, 0.03632986111, 0.03518287037, 0.03394965278, 0.03260416667, 0.03110532407, 0.02938020833, 0.0272806713, 0.02440335648}};

#define OX_PRESSURE_TABLE_LEN 20
// temperature (K) to vapour pressure (psi)
double ox_pressure_table[2][INTERPOLATION_TABLE_LENGTH] = {
    {55, 60, 65, 70, 75, 80, 85, 90, 95, 100, 105, 110, 115, 120, 125, 130, 135, 140, 145, 150},
    {0.0259, 0.10527, 0.33866, 0.90826, 2.1099, 4.369, 8.2426, 14.41, 23.653, 36.84, 54.901, 78.814, 109.59, 148.27, 195.93, 253.68, 322.72, 404.33, 500.05, 611.86}};

#define OX_MANIFOLD_TABLE_LEN 21
double ox_manifold_table[2][INTERPOLATION_TABLE_LENGTH] = {
    {192.5, 210.556, 228.611, 246.667, 264.722, 282.778, 300.833, 318.889, 336.944, 355.0, 373.056, 391.111, 409.167, 427.222, 445.278, 463.333, 481.389, 499.444, 517.5, 535.556, 550.0},
    {118.055, 127.34, 136.64, 146.082, 155.44, 165.182, 175.069, 184.896, 194.888, 205.049, 215.378, 225.702, 236.381, 246.93, 257.695, 268.683, 279.647, 290.878, 301.999, 313.482, 322.79}};

#define IPA_MANIFOLD_TABLE_LEN 21
double ipa_manifold_table[2][INTERPOLATION_TABLE_LENGTH] = {
    {192.5, 210.556, 228.611, 246.667, 264.722, 282.778, 300.833, 318.889, 336.944, 355.0, 373.056, 391.111, 409.167, 427.222, 445.278, 463.333, 481.389, 499.444, 517.5, 535.556, 550.0},
    {120.844, 130.535, 140.262, 150.16, 159.99, 170.243, 180.67, 191.054, 201.631, 212.408, 223.382, 234.368, 245.752, 257.017, 268.529, 280.3, 292.062, 304.129, 316.095, 328.466, 338.509}};

Sensor_Data default_sensor_data{
    100, // psi
    {
        820, // psi
        0,   // psi - not needed for OL
        0,   // psi - not needed for OL
        90,  // Kelvin
        90,  // Kelvin
    },
    {
        820, // psi
        0,   // psi - not needed for OL
        0,   // psi - not needed for OL
    }};

Venturi ox_venturi{0.127, 0.0204, 0.8};  // in^2 for both
Venturi ipa_venturi{0.127, 0.0204, 0.8}; // in^2 for both

// maps v from (min_in, max_in) to (min_out, max_out)
double linear_interpolation(double v, double min_in, double max_in, double min_out, double max_out) {
  return (v - min_in) / (max_in - min_in) * (max_out - min_out) + min_out;
}

// linearly interpolate using the 2 nearest values in a table
// first row of table represents input
// second row of table represents output
// if value is lower than the first value or larger than the last value, clamp to the largest or smallest output
double clamped_table_interplolation(double v, double table[2][INTERPOLATION_TABLE_LENGTH], int table_length) {
  if (v < table[0][0]) {
    return table[1][0]; // if starting value is below min, return min
  }
  for (int i = 0; i < table_length - 1; i++) {
    if (table[0][i] <= v && v < table[0][i + 1]) {
      return linear_interpolation(v, table[0][i], table[0][i + 1], table[1][i], table[1][i + 1]);
    }
  }
  return table[1][table_length - 1]; // if starting value is above max, return max
}

// get oxygen properties using temperature in Kelvin
double ox_density_from_temperature(double temperature) {
  return clamped_table_interplolation(temperature, ox_density_table, OX_DENSITY_TABLE_LEN);
}

// get ipa properties using temperature in Kelvin
double ipa_density() {
  return 0.02836; // lb/in^3
}

// The thrust coefficient (Cf) varies based on thrust
// Lookup the thrust coefficient using linear interpolation
// INPUT: thrust (lbf)
// OUTPUT: thrust coefficient (unitless)
double cf(double thrust) {
  return clamped_table_interplolation(thrust, cf_thrust_table, CF_THRUST_TABLE_LEN);
}

// convert thrust to chamber pressure using Cf equation
// INPUT: thrust (lbf)
// OUTPUT: chamber pressure (psi)
double chamber_pressure(double thrust) {
  return thrust / cf(thrust) / tadpole_AREA_OF_THROAT;
}

// convert chamber pressure to total mass flow rate using c* equation
// INPUT: chamber pressure (psi)
// OUTPUT: total mass flow rate (lbm/s)
double mass_flow_rate(double chamber_pressure) {
  return chamber_pressure * tadpole_AREA_OF_THROAT / tadpole_C_STAR * GRAVITY_FT_S;
}

// convert total mass flow into OX and IPA flow rates
// INPUT: total_mass_flow (lbm/s)
// OUTPUT: mass_flow_ox (lbm/s) and mass_flow_ipa (lbm/s)
void mass_balance(double total_mass_flow, double *mass_flow_ox, double *mass_flow_ipa) {
  *mass_flow_ox = total_mass_flow / (1 + tadpole_MASS_FLOW_RATIO) * tadpole_MASS_FLOW_RATIO;
  *mass_flow_ipa = total_mass_flow / (1 + tadpole_MASS_FLOW_RATIO);
}

// convert mass_flow into valve flow coefficient (cv)
// OUTPUT: valve flow coefficient (assume this is unitless)
// INPUT: mass_flow (lbm/s), downstream pressure (psi), fluid properties
double sub_critical_cv(double mass_flow, double upstream_pressure, double downstream_pressure, double density) {
  double pressure_delta = upstream_pressure - downstream_pressure;
  pressure_delta = pressure_delta > 0 ? pressure_delta : 0.0001; // block negative under sqrt and divide by 0
  return mass_flow * IN3_TO_GAL * PER_SEC_TO_PER_MIN * sqrt(LB_TO_TON * PER_IN3_TO_PER_M3 / pressure_delta / density);
}

// Lookup the valve angle using linear interpolation
// INPUT: valve flow coefficient (assume this is unitless)
// OUTPUT: valve angle (degrees)
double valve_angle(double cv) {
  return clamped_table_interplolation(cv, valve_angle_table, VALVE_ANGLE_TABLE_LEN);
}

// Lookup ox manifold pressure using linear interpolation
// INPUT: thrust
// OUTPUT: ox manifold pressure (psi)
double ox_manifold_pressure(double thrust) {
  return clamped_table_interplolation(thrust, ox_manifold_table, OX_MANIFOLD_TABLE_LEN);
}

// Lookup ipa manifold pressure using linear interpolation
// INPUT: thrust
// OUTPUT: ipa manifold pressure (psi)
double ipa_manifold_pressure(double thrust) {
  return clamped_table_interplolation(thrust, ipa_manifold_table, IPA_MANIFOLD_TABLE_LEN);
}

// add back other TC

// get valve angles (degrees) given thrust
void open_loop_thrust_control(double thrust, Sensor_Data sensor_data, double *angle_ox, double *angle_ipa) {
  double mass_flow_ox;
  double mass_flow_ipa;
  mass_balance(mass_flow_rate(chamber_pressure(thrust)), &mass_flow_ox, &mass_flow_ipa);

  double ox_valve_downstream_pressure_goal = ox_manifold_pressure(thrust); // TODO RJN OL - multiply these by coeff
  double ipa_valve_downstream_pressure_goal = ipa_manifold_pressure(thrust);

  *angle_ox = valve_angle(sub_critical_cv(mass_flow_ox, sensor_data.ox.tank_pressure, ox_valve_downstream_pressure_goal, ox_density_from_temperature(sensor_data.ox.venturi_temperature)));
  *angle_ipa = valve_angle(sub_critical_cv(mass_flow_ipa, sensor_data.ipa.tank_pressure, ipa_valve_downstream_pressure_goal, ipa_density()));
}

void open_loop_thrust_control_defaults(double thrust, double *angle_ox, double *angle_ipa) {
  open_loop_thrust_control(thrust, default_sensor_data, angle_ox, angle_ipa);
}

// Estimates mass flow across a venturi using pressure sensor data and fluid information.
double estimate_mass_flow(Fluid_Line fluid_line, Venturi venturi, double fluid_density) {
  double pressure_delta = fluid_line.venturi_upstream_pressure - fluid_line.venturi_throat_pressure;
  pressure_delta = pressure_delta > 0 ? pressure_delta : 0; // block negative under sqrt
  double area_term = 1 - pow(venturi.throat_area / venturi.inlet_area, 2);
  return venturi.throat_area * sqrt(2 * fluid_density * pressure_delta / (1 - area_term)) * venturi.cd;
}

void closed_loop_thrust_control(double thrust, double time_delta, double mfr_ox, double mfr_ipa, double chamber_pressure_sensor,
                                double *cp_err_sum, double *ox_err_sum, double *ipa_err_sum, double *angle_ox, double *angle_ipa) {
  // ol_ for open loop computations
  // err_ for err between ol and sensor
  // col_ for closed loop computation

  double ol_chamber_pressure = chamber_pressure(thrust);
  double err_chamber_pressure = chamber_pressure_sensor - ol_chamber_pressure;
  double ol_mdot_total = mass_flow_rate(ol_chamber_pressure);
  double cl_mdot_total = ol_mdot_total - ClosedLoopControllers::Chamber_Pressure_Controller.compute(err_chamber_pressure, time_delta, cp_err_sum);

  double ol_mass_flow_ox;
  double ol_mass_flow_ipa;
  mass_balance(cl_mdot_total, &ol_mass_flow_ox, &ol_mass_flow_ipa);

  double err_mass_flow_ox = mfr_ox - ol_mass_flow_ox;
  double err_mass_flow_ipa = mfr_ipa - ol_mass_flow_ipa;

  double ox_valve_downstream_pressure_goal = ox_manifold_pressure(thrust);
  double ipa_valve_downstream_pressure_goal = ipa_manifold_pressure(thrust);

  double ol_angle_ox = valve_angle(sub_critical_cv(ol_mass_flow_ox, default_sensor_data.ox.tank_pressure, ox_valve_downstream_pressure_goal, ox_density_from_temperature(default_sensor_data.ox.valve_temperature)));
  double ol_angle_ipa = valve_angle(sub_critical_cv(ol_mass_flow_ipa, default_sensor_data.ipa.tank_pressure, ipa_valve_downstream_pressure_goal, ipa_density()));

  *angle_ox = ol_angle_ox - ClosedLoopControllers::LOX_Angle_Controller.compute(err_mass_flow_ox, time_delta, ox_err_sum);
  *angle_ipa = ol_angle_ipa - ClosedLoopControllers::IPA_Angle_Controller.compute(err_mass_flow_ipa, time_delta, ipa_err_sum);
}