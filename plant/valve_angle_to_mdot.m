function mdot = valve_angle_to_mdot(valve_angle, tank_pressure, manifold_pressure, fluid_density)
    cv_table = [
        0, 9, 18, 27, 36, 45, 54, 63, 72, 81, 90;
        0.000, 0.070, 0.161, 0.378, 0.670, 1.000, 1.450, 2.050, 2.780, 3.710, 4.960
    ];

    IN3_TO_GAL = 0.004329;       % convert cubic inches to gallons
    PER_SEC_TO_PER_MIN = 60;     % convert per second to per minute
    LB_TO_TON = 0.000453592;     % convert lb to metric tons
    PER_IN3_TO_PER_M3 = 61023.7; % convert per in^3 to per m^3

    pressure_delta = tank_pressure - manifold_pressure;
    pressure_delta = max(0, pressure_delta); % block negative under sqrt and divide by 0
    cv = clamped_interpolation(valve_angle, cv_table)
    mdot = cv / IN3_TO_GAL / PER_SEC_TO_PER_MIN * sqrt(fluid_density * pressure_delta / LB_TO_TON / PER_IN3_TO_PER_M3);
end