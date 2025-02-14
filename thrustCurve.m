function thrust = thrustCurve(time)

maxT = 550;
if time < 10
    thrust = maxT*(1 - 1 / (5/3 + exp(30 - 4*time)));
elseif time < 20
    thrust = maxT*(0.4 + 1 / (5/3 + exp(50 - 4*time)));
else
    thrust = maxT;

end

