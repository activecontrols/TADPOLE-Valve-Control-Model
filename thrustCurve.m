function thrust = thrustCurve(time)

CURVE_TIME = 20;
SLOPE = 2.5;
OFFSET = 6 * SLOPE;


maxT = 550;
if time < CURVE_TIME / 2
    thrust = maxT*(1 - 1 / (20/13 + exp(OFFSET - SLOPE*time)));
elseif time < CURVE_TIME
    thrust = maxT*(1 - 1 / (20/13 + exp(OFFSET - SLOPE*(CURVE_TIME-time))));
else
    thrust = maxT;
end

