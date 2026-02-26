% Ziegler-Nichols closed-loop PID tuning in MATLAB

Ku = 6.0;     % ultimate gain (measured experimentally)
Tu = 1.2;     % ultimate period (seconds)

% PID controller parameters (closed-loop ZN)
Kp = 0.60 * Ku;
Ti = Tu / 2.0;
Td = Tu / 8.0;

Ki = Kp / Ti;
Kd = Kp * Td;

fprintf('Kp = %.3f, Ki = %.3f, Kd = %.3f\n', Kp, Ki, Kd);

% Example plant: DC motor-like first-order model
Kplant = 1.5;
tau = 0.4;
s = tf('s');
Gp = Kplant / (tau * s + 1);

C = Kp + Ki / s + Kd * s;   % PID in parallel form
Tcl = feedback(C*Gp, 1);    % unity-feedback closed loop

figure;
step(Tcl);
grid on;
title('Ziegler-Nichols PID step response');

% Simulink:
% - Use a "PID Controller" block in parallel form.
% - Enter Kp, Ki, Kd computed above.
% - Connect plant model Gp (e.g., using a Transfer Fcn block).
