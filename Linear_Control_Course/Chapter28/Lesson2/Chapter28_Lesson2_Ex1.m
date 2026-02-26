simOut = sim('SecondOrderServoModel');  % name of the Simulink model file
t_sim  = simOut.tout;
y_sim  = simOut.yout.signals.values;
