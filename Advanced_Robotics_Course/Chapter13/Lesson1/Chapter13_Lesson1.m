% Parameters used for controller tuning
Kp = 5.0;
Kd = 1.0;
c_sim  = 0.1;   % nominal damping
c_real = 0.3;   % true damping to emulate the robot

% Open the Simulink model (assumes it exists on path)
open_system('sim2real_1dof');

% Configure controller gains (e.g., in a "Gain" block)
set_param('sim2real_1dof/Controller/Kp', 'Gain', num2str(Kp));
set_param('sim2real_1dof/Controller/Kd', 'Gain', num2str(Kd));

% First simulate with nominal damping
set_param('sim2real_1dof/Plant', 'c', num2str(c_sim));
simOutSim = sim('sim2real_1dof', 'StopTime', '5.0');
thetaSim = simOutSim.logsout.get('theta').Values.Data;

% Now simulate with "real" damping
set_param('sim2real_1dof/Plant', 'c', num2str(c_real));
simOutReal = sim('sim2real_1dof', 'StopTime', '5.0');
thetaReal = simOutReal.logsout.get('theta').Values.Data;

% Compare trajectories
t = simOutSim.tout;
figure;
plot(t, thetaSim, 'LineWidth', 1.5); hold on;
plot(t, thetaReal, '--', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('\theta (rad)');
legend('Nominal model', 'Mismatched model');
title('Closed-loop response with and without damping mismatch');
grid on;
      
