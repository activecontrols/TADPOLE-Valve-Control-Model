# TADPOLE-Valve-Control-Model
Here you can find the Simulink and MATLAB Code for the Closed Loop Control of TADPOLE's valves

This is the first version of the Model. It contains 3 PI controllers with feedforwards for the closed loop control.
The plant is equipped with noise models, modeling errors that are randomized, and basic actuator dynamics including delays.
Feel free to play around with tuning the gains for the Integral and Proportional terms of the controller, they are 
located in the MATLAB file. Be sure to run that script before running the simulink model to apply changes. 

NOTE: Controller seems to jitter more when proportional terms are added. There is a chance we move forward with pure integral control (no P gain).

If you have any questions, DM me on Slack: @pabloplataa

Next steps would involve using data from water flow tests and/or future static testing to create more accurate plant dynamics and noise/error models
