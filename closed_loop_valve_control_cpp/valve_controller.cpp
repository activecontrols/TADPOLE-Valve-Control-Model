#include "valve_controller.hpp"
#include "pi_controller.hpp"
#include "math.h"

#define tadpole_AREA_OF_THROAT 1.69 // in^2
#define tadpole_MASS_FLOW_RATIO 1.2 // #ox = 1.2 * ipa
#define GRAVITY_FT_S 32.1740        // Gravity in (ft / s^2)

#define IN3_TO_GAL 0.004329     // convert cubic inches to gallons
#define PER_SEC_TO_PER_MIN 60   // convert per second to per minute
#define DENSITY_WATER 0.0360724 // lb/in^3

#define OX_INJ_AREA 0.0498   // in^2
#define IPA_INJ_AREA 0.04031 // in^2
#define OX_INJ_CD 0.445      // TODO RJN - change during hotfires
#define IPA_INJ_CD 0.7

#define INTERPOLATION_TABLE_LENGTH 30 // max length of all tables - set to enable passing tables to functions

#define CF_THRUST_TABLE_LEN 2 // TODO RJN OL - replace with data from testing
// thrust (lbf) to cf (unitless)
double cf_thrust_table[2][INTERPOLATION_TABLE_LENGTH] = {
    {220, 550},
    {1.12, 1.3}};

#define CSTAR_CHAMBER_PRESSURE_TABLE_LEN 2
// thrust (lbf) to cf (unitless)
double cstar_chamber_pressure_table[2][INTERPOLATION_TABLE_LENGTH] = {
    {100, 250},
    {4455, 3857}};

#define OX_DENSITY_TABLE_LEN 20
// temperature (K) to density (lb/in^3)
double ox_density_table[2][INTERPOLATION_TABLE_LENGTH] = {
    {55, 60, 65, 70, 75, 80, 85, 90, 95, 100, 105, 110, 115, 120, 125, 130, 135, 140, 145, 150},
    {0.04709027778, 0.04631539352, 0.04550925926, 0.0446880787, 0.04385474537, 0.04300810185, 0.04214525463, 0.04126099537, 0.04035127315, 0.03941087963, 0.03843229167, 0.03740856481, 0.03632986111, 0.03518287037, 0.03394965278, 0.03260416667, 0.03110532407, 0.02938020833, 0.0272806713, 0.02440335648}};

#define IPA_CV_TABLE_LEN 11
double ipa_valve_cv_table[2][INTERPOLATION_TABLE_LENGTH] = {
    {0.095, 0.130, 0.222, 0.336, 0.469, 0.640, 0.868, 1.164, 1.507, 1.836, 2.029},
    {25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75}};

#define OX_CV_TABLE_LEN 12
double ox_valve_cv_table[2][INTERPOLATION_TABLE_LENGTH] = {
    {0.084, 0.143, 0.237, 0.366, 0.531, 0.730, 0.960, 1.217, 1.495, 1.787, 2.084, 2.378},
    {25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80}};

Venturi ox_venturi{0.127, 0.066, 1};  // in^2 for both
Venturi ipa_venturi{0.127, 0.062, 1}; // in^2 for both

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
  return chamber_pressure * tadpole_AREA_OF_THROAT / clamped_table_interplolation(chamber_pressure, cstar_chamber_pressure_table, CSTAR_CHAMBER_PRESSURE_TABLE_LEN) * GRAVITY_FT_S;
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
  return mass_flow * IN3_TO_GAL * PER_SEC_TO_PER_MIN * sqrt(1 / (pressure_delta * density * DENSITY_WATER));
}

// Lookup the valve angle using linear interpolation
// INPUT: valve flow coefficient (assume this is unitless)
// OUTPUT: valve angle (degrees)
double ipa_valve_angle(double cv) {
  return clamped_table_interplolation(cv, ipa_valve_cv_table, IPA_CV_TABLE_LEN);
}

// Lookup the valve angle using linear interpolation
// INPUT: valve flow coefficient (assume this is unitless)
// OUTPUT: valve angle (degrees)
double lox_valve_angle(double cv) {
  return clamped_table_interplolation(cv, ox_valve_cv_table, OX_CV_TABLE_LEN);
}

double manifold_drop(double target_mass_flow, double density, double injector_area, double c_d) {
  return pow(target_mass_flow, 2) / (2 * density * GRAVITY_FT_S * 12 * pow(c_d * injector_area, 2));
}

// Estimates mass flow across a venturi using pressure sensor data and fluid information.
double estimate_mass_flow(Fluid_Line fluid_line, Venturi venturi, double fluid_density) {
  double pressure_delta = fluid_line.venturi_differential_pressure;
  pressure_delta = pressure_delta > 0 ? pressure_delta : 0; // block negative under sqrt
  double area_term = pow(venturi.throat_area / venturi.inlet_area, 2);
  return venturi.throat_area * sqrt(2 * fluid_density * pressure_delta * 12 * GRAVITY_FT_S / (1 - area_term)) * venturi.cd;
}

// get valve angles (degrees) given thrust (lbf) and current sensor data
void open_loop_thrust_control(double thrust, double ox_valve_upstream, double ipa_valve_upstream, double *angle_ox, double *angle_ipa) {
  double mass_flow_total = mass_flow_rate(chamber_pressure(thrust));
  double mass_flow_ox;
  double mass_flow_ipa;
  mass_balance(mass_flow_total, &mass_flow_ox, &mass_flow_ipa);

  double ox_manifold_drop = manifold_drop(mass_flow_ox, ox_density_from_temperature(90), OX_INJ_AREA, OX_INJ_CD);
  double ipa_manifold_drop = manifold_drop(mass_flow_ipa, ipa_density(), IPA_INJ_AREA, IPA_INJ_CD);
  double ox_valve_downstream_pressure_goal = chamber_pressure(thrust) + ox_manifold_drop;
  double ipa_valve_downstream_pressure_goal = chamber_pressure(thrust) + ipa_manifold_drop;

  *angle_ox = lox_valve_angle(sub_critical_cv(mass_flow_ox, ox_valve_upstream, ox_valve_downstream_pressure_goal, ox_density_from_temperature(90)));
  *angle_ipa = ipa_valve_angle(sub_critical_cv(mass_flow_ipa, ipa_valve_upstream, ipa_valve_downstream_pressure_goal, ipa_density()));
}

// get valve angles (degrees) given thrust (lbf) and current sensor data using PID controllers
void closed_loop_thrust_control(double thrust, double time_delta, double mfr_ox, double mfr_ipa, double chamber_pressure_sensor,
                                double ox_valve_upstream, double ipa_valve_upstream, double *cp_err_sum, double *ox_err_sum, double *ipa_err_sum,
                                double *angle_ox, double *angle_ipa) {

  // ol_ for open loop computations
  // err_ for err between ol and sensor
  // col_ for closed loop computation

  double measured_mass_flow_ox = mfr_ox;
  double measured_mass_flow_ipa = mfr_ipa;

  double ol_chamber_pressure = chamber_pressure(thrust);
  double err_chamber_pressure = chamber_pressure_sensor - ol_chamber_pressure;
  double ol_mdot_total = mass_flow_rate(ol_chamber_pressure);
  double cl_mdot_total = ol_mdot_total - ClosedLoopControllers::Chamber_Pressure_Controller.compute(err_chamber_pressure, time_delta, cp_err_sum);

  double ol_mass_flow_ox;
  double ol_mass_flow_ipa;
  mass_balance(cl_mdot_total, &ol_mass_flow_ox, &ol_mass_flow_ipa);

  double err_mass_flow_ox = measured_mass_flow_ox - ol_mass_flow_ox;
  double err_mass_flow_ipa = measured_mass_flow_ipa - ol_mass_flow_ipa;

  double ox_manifold_drop = manifold_drop(ol_mass_flow_ox, ox_density_from_temperature(90), OX_INJ_AREA, OX_INJ_CD);
  double ipa_manifold_drop = manifold_drop(ol_mass_flow_ipa, ipa_density(), IPA_INJ_AREA, IPA_INJ_CD);
  double ox_valve_downstream_pressure_goal = chamber_pressure(thrust) + ox_manifold_drop;
  double ipa_valve_downstream_pressure_goal = chamber_pressure(thrust) + ipa_manifold_drop;

  double ol_angle_ox = lox_valve_angle(sub_critical_cv(ol_mass_flow_ox, ox_valve_upstream, ox_valve_downstream_pressure_goal, ox_density_from_temperature(90)));
  double ol_angle_ipa = ipa_valve_angle(sub_critical_cv(ol_mass_flow_ipa, ipa_valve_upstream, ipa_valve_downstream_pressure_goal, ipa_density()));

  *angle_ox = ol_angle_ox - ClosedLoopControllers::LOX_Angle_Controller.compute(err_mass_flow_ox, time_delta, ox_err_sum);
  *angle_ipa = ol_angle_ipa - ClosedLoopControllers::IPA_Angle_Controller.compute(err_mass_flow_ipa, time_delta, ipa_err_sum);
}