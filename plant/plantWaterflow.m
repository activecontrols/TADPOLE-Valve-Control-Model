function xdot = plantWaterflow(x, angle_ox, cvCOEFS, P_tank)
    %% Curve fit V30 valve
    % Cv given angle
    % cvOX = polyval(cvCOEFS, angle_ox);
    cvOX = 2.55 ./ (1 + exp(-(angle_ox - 60) / 10));

    %States
    P_out = x(1);
    mdot = x(2);

    %Data
    water_density = 0.0361;     %lb/in^3

    %% Parameters
    % Valve settling times
    tau_valve = 0.01;

    % State Derivative
    xdot = zeros(2, 1);
    %% ODEs
    rho_fluid = water_density;
    rhoWat = water_density;
    P_down = 14.7;
    C_d = 1.63;        %1.355
    A_e = 0.127;
    A_i = A_e;
    A_th = 0.0203;
    l = 10;
    K = 44700;
    V_d = A_e * l;
    g = 32.174 * 12;

    % Numerical
    mdot_in = 231/60 * cvOX * sqrt(rho_fluid * rhoWat * (P_tank - P_out));
    DPVenturi = mdot_in.^2 * (1 - (A_th / A_e)^2) / (2 * A_th^2 * rho_fluid * g);
    mdot_out = C_d * A_i * sqrt(2 * rho_fluid * max((P_out - DPVenturi - P_down), 0));

    xdot(1) = K / V_d * (mdot_in - mdot_out);
    xdot(2) = (-mdot + mdot_in) / tau_valve;
end

