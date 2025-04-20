function xdot = plantWaterflow(x, angle_ox, cvCOEFS)

    syms P_out_dot P_out_S mdot_in_S mdot_out_S
    %% Curve fit V30 valve
    % Cv given angle
    cvOX = -9.3261e-08*angle_ox^3 + 2.9512e-04*angle_ox^2 + 6.2035e-04*angle_ox;
    cvOX = polyval(cvCOEFS, angle_ox);

    %States
    P_out = x(1);
    mdot = x(2);

    %Data
    water_density = 0.0361;     %lb/in^3

    %% Parameters
    % Valve settling times
    tau_valve = 0.1;

    % State Derivative
    xdot = zeros(2, 1);
    %% ODEs
    rho_fluid = water_density;
    rhoWat = water_density;
    P_atm = 0;
    P_tank = 80;
    C_d = 1.35;
    A_e = 0.127;
    l = 10;
    K = 1000;
    V_d = A_e * l;

    % Numerical
    mdot_in = 231/60 * cvOX * sqrt(rho_fluid * rhoWat * (P_tank - P_out));
    mdot_out = C_d * A_e * sqrt(2 * rho_fluid * max((P_out - P_atm), 0));

    xdot(1) = K / V_d * (mdot_in - mdot_out);
    xdot(2) = (-mdot + mdot_in) / tau_valve;
end

