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
    P_down = 14.3;
    C_d = 0.366;        %1.355
    A_l = 0.127;
    A_i = 0.02544;
    A_th = 0.0203;
    l = 10;
    K = 44700;
    V_d = A_l * l;
    g = 32.174 * 12;
    C_fric = 0.03;
    d = 0.35;

    % Numerical
    mdot_in = 231/60 * cvOX * sqrt(rho_fluid * rhoWat * (P_tank - P_out));

    DP_i = 1 / (2 * rho_fluid * g *(C_d * A_i)^2);
    % DP_l = C_fric * l / (2 * d * g * rho_fluid * A_l^2);

    % DPVenturi = (1 - (A_th / A_l)^2) / (2 * A_th^2 * rho_fluid * g);

    xdot(1) = K / V_d * (mdot_in - mdot);
    xdot(2) = (P_out - P_down - (DP_i) * mdot^2) * g * (A_l/l);
end

