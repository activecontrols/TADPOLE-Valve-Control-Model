function mdot = oxangle2mdot(angle, consts, simData, fuelData)

    % Conversion Factors and LookUp Arrays
    conversionFactors = [0.004329 60 0.000453592 61023.7];
    subCritVArray = [0.000, 0.070, 0.161, 0.378, 0.670, 1.000, 1.450, 2.050, 2.780, 3.710, 4.960];
    valveAngleArray = [0, 9, 18, 27, 36, 45, 54, 63, 72, 81, 90];
    tempArray = [55, 60, 65, 70, 75, 80, 85, 90, 95, 100, 105, 110, ...
    115, 120, 125, 130, 135, 140, 145, 150];
    oxDensityArray = fuelData(1,:);
    oxPressureArray = fuelData(2,:);

    % cV lookup based off of angle
    subCritV = interp1(valveAngleArray, subCritVArray, angle);

    % ox
    throat = 0.00989;
    cd = 0.975;

    % density and vapour pressure ox
    simData(4);
    density_cv = interp1(tempArray, oxDensityArray, simData(4));
    p_vap = interp1(tempArray, oxPressureArray, simData(4));
    density_valve = interp1(tempArray, oxDensityArray, simData(3));

    %Generates mdot
    alpha = conversionFactors(1)*conversionFactors(2);
    gamma = cd^2*throat^2*2*density_cv*(consts(3)*12);
    beta = conversionFactors(3)*conversionFactors(4);
    deltaP = simData(1)-p_vap;
    mdot = sqrt(subCritV^2*density_valve*(deltaP)*gamma /...
        (alpha^2*gamma*beta + subCritV^2*density_valve));
end

