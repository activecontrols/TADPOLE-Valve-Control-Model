function xdot = nonlinear_plant3(x, angle_ox, angle_ipa, t, distMode)

    %States
    distMode = 0;
    mdot_ox = x(1) * (1 + distMode* (0.01 + 0.01 * sin(2*t)));
    mdot_ipa = x(2) * (1 + distMode* (0.01 + 0.01 * sin(2*t)));
    Pc = x(3) * (1 + distMode* (0.01 + 0.01 * sin(2*t)));
    P_out_ox = x(4);
    P_out_ipa = x(5);

    %% PARAMS
    distMode = 0;
    t = 0;
    P_tank_ox = 820 + distMode * 2*sin(100*t); % psi, with disturbance
    P_tank_ipa = 820 + distMode * 2*sin(155*t); % psi, with disturbance
    ox_density = 71.1936 / 1728; %lb/in^3
    ipa_density = 49.06838 / 1728; %lb/in^3
    water_density = 0.0361;     %lb/in^3

    OF = mdot_ox / max(mdot_ipa, 1e-10);

    A_t = 1.697;                            %in^2
    V_c = 67.3196;                          %in^3
    R_u = 10.7315 * 12^2;                   %psi*ft^3*lb/mol/°R
    R = R_u / 18.8;                         %psi*ft^3*lb/mol/°R 
    T_c = tempfcn(OF);                      %Rankine
    g = 32.174 * 12;                        %in/s
    cstar = fcn_cstar(OF);                  %ft/s
    d = (0.5 - 2*0.065);                    %diameter of line
    A_l = pi * (d / 2)^2;                   %in^2
    l_f = 120;                              %in
    l_o = 240;                              %in
    A_io = 0.04031006898;                   %in^2
    A_if = 0.04875965463;                   %in^2
    C_fric = 0.03;                          %line friction coef
    C_d = 0.79;                              %injector coef
    l_eq_ox = 7;                            %eq line length ox
    l_eq_ipa = 7;                           %eq line length ipa
    V_lo = A_l * l_eq_ox;
    V_lf = A_l * l_eq_ox;
    
    % Line and injector pressure drops
    DP_i_ox = 1 / (2 * ox_density * g *(C_d * A_io)^2);
    DP_l_ox = C_fric * l_eq_ox / (2 * d * g * ox_density * A_l^2);

    DP_i_ipa = 1 / (2 * ipa_density * g *(C_d * A_if)^2);
    DP_l_ipa = C_fric * l_eq_ipa / (2 * d * g * ipa_density * A_l^2);

    % Valve Cvs
    cvOX = 2.55 ./ (1 + exp(-(angle_ox - 60) / 10));
    cvIPA = 2.55 ./ (1 + exp(-(angle_ipa - 60) / 10));

    % Fluid build up inside line
    mdot_in_ox = 231/60 * cvOX * sqrt(ox_density * water_density * max(P_tank_ox - P_out_ox, 0));
    mdot_in_ipa = 231/60 * cvIPA * sqrt(ipa_density * water_density * max(P_tank_ipa - P_out_ipa, 0));
    
    % State Derivative
    xdot = zeros(5,1);

    %% ODEs

    xdot(1) = (P_out_ox - Pc - (DP_i_ox + DP_l_ox) * mdot_ox^2) * g * (A_l/l_o);
    xdot(2) = (P_out_ipa - Pc - (DP_i_ipa + DP_l_ipa) * mdot_ipa^2) * g * (A_l/l_f);
    xdot(3) = (R * T_c / V_c)*(mdot_ox + mdot_ipa - A_t*(g /12) / cstar * Pc); 
    xdot(4) = R * 162 / V_lo * (mdot_in_ox - mdot_ox);
    xdot(5) = R * 527 / V_lf * (mdot_in_ipa - mdot_ipa);

end

    