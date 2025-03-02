function thrust = test4(time)

% Closed Loop Fast Command Response

maxT = 550;
if time < 2
    thrust = maxT;
elseif time < 5.5
    thrust = stdCurve(time,7,8,3,0.8);
elseif time < 10.5
    thrust = stdCurve(time,15.75,5,7,0.65);
elseif time < 14
    thrust = stdCurve(time,25,9,11.65,0.5);
elseif time < 19
    thrust = stdCurve(time, 38, 3.5, 16.25, 0.35);
else 
    thrust = 0.35*maxT;
end
