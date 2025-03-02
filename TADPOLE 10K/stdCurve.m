function thrust = stdCurve(time, CURVE_TIME, SLOPE, OFFSET, MIN)

% Generates a standard thrust curve with the specified params
maxT = 550;
if time < CURVE_TIME / 2
    thrust = maxT*(1 - 1 / ((1 - MIN)^(-1) + exp(OFFSET*SLOPE - SLOPE*time)));
elseif time < CURVE_TIME
    thrust = maxT*(1 - 1 / ((1 - MIN)^(-1) + exp(OFFSET*SLOPE - SLOPE*(CURVE_TIME-time))));
else
    thrust = maxT;
end
