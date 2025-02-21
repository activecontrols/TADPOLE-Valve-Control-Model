function xdot = NonLinearModel(x, angle_ox, angle_ipa, consts, simData, fuelData)
    
    % States 
    mdot_ox = x(1);
    mdot_ipa = x(2);
    Pc = x(3);
    angle_ox = min(max(angle_ox, 0), 89.99);
    angle_ipa = min(max(angle_ipa, 0), 89.99);

    % Time constants
    tau_mdot = 0.11 / 4;
    tau_pc = 1.5 / 4;

    % Nonlinear mass flow rates
    f_ox = oxangle2mdot(angle_ox, consts, simData, fuelData);
    f_ipa = ipaangle2mdot(angle_ipa, consts, simData);

    % Extract model errors from consts vector
    oxError = consts(5);
    ipaError = consts(6);
    PcError = consts(7);

    % State derivatives
    xdot = zeros(3,1);
    xdot(1) = (-mdot_ox + f_ox * oxError) / tau_mdot;
    xdot(2) = (-mdot_ipa + f_ipa * ipaError) / tau_mdot;
    xdot(3) = (-Pc + (mdot_ox + mdot_ipa) * PcError * consts(2) / (consts(1) * consts(3)))...
        / tau_pc;

end

