function thrust = thrustCurve(time)

CURVE_TIME = 22.5;
SLOPE = 3.25;
OFFSET = 8.75;
MIN = 0.35;

thrust = stdCurve(time, CURVE_TIME, SLOPE, OFFSET, MIN);
end

