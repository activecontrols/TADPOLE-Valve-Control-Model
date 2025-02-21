function mdot2 = ipaangle2mdot(angle, consts, simData)

    % Conversion Factors and LookUp Arrays
    conversionFactors = [0.004329 60 0.000453592 61023.7];
    subCritVArray = [0.000, 0.070, 0.161, 0.378, 0.670, 1.000, 1.450, 2.050, 2.780, 3.710, 4.960];
    valveAngleArray = [0, 9, 18, 27, 36, 45, 54, 63, 72, 81, 90];
    ipaDensityArray = 0.02836;
    ipaPressureArray = 0.638;

    % cV lookup based off of angle
    subCritV = interp1(valveAngleArray, subCritVArray, angle);

    % ox
    throat = 0.00989;
    cd = 0.975;

    % density and vapour pressure ox
    simData(4);
    density_cv = ipaDensityArray;
    p_vap = ipaPressureArray;
    density_valve = ipaDensityArray;

    %Generates mdot
    alpha = conversionFactors(1)*conversionFactors(2);
    gamma = cd^2*throat^2*2*density_cv*(consts(3)*12);
    beta = conversionFactors(3)*conversionFactors(4);
    deltaP = simData(1)-p_vap;
    mdot2 = sqrt(subCritV^2*density_valve*(deltaP)*gamma /...
        (alpha^2*gamma*beta + subCritV^2*density_valve));
end