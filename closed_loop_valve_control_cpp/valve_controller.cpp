#include "valve_controller.hpp"
#include "pi_controller.hpp"
#include <math.h>

// TODO RJN open loop valve - efficiency coeff?
#define tadpole_AREA_OF_THROAT 1.69      // in^2
#define tadpole_C_STAR 4998.0654         // ft / s // TODO RJN open loop valve - vary based on thrust
#define tadpole_MASS_FLOW_RATIO 1.2      // #ox = 1.2 * fuel
#define GRAVITY_FT_S 32.1740             // Gravity in (ft / s^2)
#define GRAVITY_IN_S (GRAVITY_FT_S * 12) // Gravity in (in / s^2)

#define DEFAULT_CHAMBER_PRESSURE 100 // psi

#define cav_vent_ox_AREA_OF_THROAT 0.00989 // in^2 #-200 deg C
#define cav_vent_ox_CD 0.975               // unitless
#define fluid_ox_DEFAULT_TEMPERATURE 90    // Kelvin
#define fluid_ox_UPSTREAM_PRESSURE 820     // psi

#define cav_vent_ipa_AREA_OF_THROAT 0.00989 // in^2 #room temp
#define cav_vent_ipa_CD 0.975               // unitless
#define fluid_ipa_DEFAULT_TEMPERATURE 280   // Kelvin
#define fluid_ipa_UPSTREAM_PRESSURE 820     // psi

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

#define CF_THRUST_TABLE_LEN 2
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

sensor_data default_sensor_data{DEFAULT_CHAMBER_PRESSURE, fluid_ox_UPSTREAM_PRESSURE, fluid_ipa_UPSTREAM_PRESSURE, fluid_ox_DEFAULT_TEMPERATURE, fluid_ipa_DEFAULT_TEMPERATURE, fluid_ox_DEFAULT_TEMPERATURE, fluid_ipa_DEFAULT_TEMPERATURE};
cav_vent ox_cv{cav_vent_ox_AREA_OF_THROAT, cav_vent_ox_CD};
cav_vent ipa_cv{cav_vent_ipa_AREA_OF_THROAT, cav_vent_ipa_CD};

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
fluid ox_from_temperature(double temperature) {
  fluid ox_data;
  ox_data.density = clamped_table_interplolation(temperature, ox_density_table, OX_DENSITY_TABLE_LEN);
  ox_data.vapour_pressure = clamped_table_interplolation(temperature, ox_pressure_table, OX_PRESSURE_TABLE_LEN);
  return ox_data;
}

// get ipa properties using temperature in Kelvin
fluid ipa_from_temperature(double temperature) {
  fluid ipa_data;
  ipa_data.density = 0.02836;       // lb/in^3 // TODO RJN open loop valve - assume constant
  ipa_data.vapour_pressure = 0.638; // psi // TODO RJN open loop valve - depends on temperature
  return ipa_data;
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

// convert total mass flow into OX and FUEL flow rates
// INPUT: total_mass_flow (lbm/s)
// OUTPUT: mass_flow_ox (lbm/s) and mass_flow_fuel (lbm/s)
void mass_balance(double total_mass_flow, double *mass_flow_ox, double *mass_flow_fuel) {
  *mass_flow_ox = total_mass_flow / (1 + tadpole_MASS_FLOW_RATIO) * tadpole_MASS_FLOW_RATIO;
  *mass_flow_fuel = total_mass_flow / (1 + tadpole_MASS_FLOW_RATIO);
}

// convert mass_flow into pressure using cavitating venturi equation
// INPUT: mass_flow (lbm/s), fluid properties
// OUTPUT: pressure (psi)
double cavitating_venturi(double mass_flow, cav_vent cvProperties, fluid cvFluid) {
  return pow(mass_flow / cvProperties.cav_vent_throat_area / cvProperties.cav_vent_cd, 2) / 2 / GRAVITY_IN_S / cvFluid.density + cvFluid.vapour_pressure;
}

// convert mass_flow into valve flow coefficient (cv)
// OUTPUT: valve flow coefficient (assume this is unitless)
// INPUT: mass_flow (lbm/s), downstream pressure (psi), fluid properties
double sub_critical_cv(double mass_flow, double upstream_pressure, double downstream_pressure, fluid cvFluid) {
  return mass_flow * IN3_TO_GAL * PER_SEC_TO_PER_MIN * sqrt(LB_TO_TON * PER_IN3_TO_PER_M3 / (upstream_pressure - downstream_pressure) / cvFluid.density);
}

// Lookup the valve angle using linear interpolation
// INPUT: valve flow coefficient (assume this is unitless)
// OUTPUT: valve angle (degrees)
double valve_angle(double cv) {
  return clamped_table_interplolation(cv, valve_angle_table, VALVE_ANGLE_TABLE_LEN);
}

void closed_loop_thrust_control(double thrust, double time_delta, double mfr_ox, double mfr_ipa, double *angle_ox, double *angle_fuel) {
  // ol_ for open loop computations
  // err_ for err between ol and sensor
  // col_ for closed loop computation

  double ol_chamber_pressure = chamber_pressure(thrust);
  double err_chamber_pressure = default_sensor_data.chamber_pressure - ol_chamber_pressure;
  double cl_chamber_pressure = ol_chamber_pressure + ClosedLoopControllers::Chamber_Pressure_Controller.compute(err_chamber_pressure, time_delta);

  double ol_mass_flow_ox;
  double ol_mass_flow_fuel;
  mass_balance(mass_flow_rate(cl_chamber_pressure), &ol_mass_flow_ox, &ol_mass_flow_fuel);

  double err_mass_flow_ox = mfr_ox - ol_mass_flow_ox;
  double err_mass_flow_fuel = mfr_ipa - ol_mass_flow_fuel;

  double ox_valve_downstream_pressure_goal = cavitating_venturi(ol_mass_flow_ox, ox_cv, ox_from_temperature(default_sensor_data.ox_cv_temperature));
  double ipa_valve_downstream_pressure_goal = cavitating_venturi(ol_mass_flow_fuel, ipa_cv, ipa_from_temperature(default_sensor_data.ipa_cv_temperature));
  double ol_angle_ox = valve_angle(sub_critical_cv(ol_mass_flow_ox, default_sensor_data.ox_tank_pressure, ox_valve_downstream_pressure_goal, ox_from_temperature(default_sensor_data.ox_valve_temperature)));
  double ol_angle_fuel = valve_angle(sub_critical_cv(ol_mass_flow_fuel, default_sensor_data.ipa_tank_pressure, ipa_valve_downstream_pressure_goal, ipa_from_temperature(default_sensor_data.ipa_valve_temperature)));

  *angle_ox = ol_angle_ox + ClosedLoopControllers::LOX_Angle_Controller.compute(err_mass_flow_ox, time_delta);
  *angle_fuel = ol_angle_fuel + ClosedLoopControllers::LOX_Angle_Controller.compute(err_mass_flow_ox, time_delta);
}