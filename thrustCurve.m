function thrust = thrustCurve(time)

CURVE_TIME = 22.5;
SLOPE = 3.25;
OFFSET = 8.75 * SLOPE;


maxT = 550;
if time < CURVE_TIME / 2
    thrust = maxT*(1 - 1 / (20/13 + exp(OFFSET - SLOPE*time)));
elseif time < CURVE_TIME
    thrust = maxT*(1 - 1 / (20/13 + exp(OFFSET - SLOPE*(CURVE_TIME-time))));
else
    thrust = maxT;
end

