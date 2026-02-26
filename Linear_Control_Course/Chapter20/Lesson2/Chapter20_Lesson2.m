s = tf('s');

% Plant: simple velocity servo model
P = 1 / ((0.5 * s + 1) * s);  % type-1 plant

% PID gains from a previous 1-DOF design
Kp = 2.0;
Ki = 5.0;
Kd = 0.1;

% Set-point weights
b = 0.6;
c = 0.0;

% 1-DOF PID transfer function
C = Kp + Ki / s + Kd * s;

% Reference path transfer function F_r(s)
Fr = Kp * b + Ki / s + Kd * c * s;

% Closed-loop transfer functions using analytical formulas
Gyr = (P * Fr) / (1 + P * C);  % from r to y
Gyd = P / (1 + P * C);         % from d (load) to y

figure;
step(Gyr);
hold on;
step(Gyd);
grid on;
legend('G_{yr}(s)', 'G_{yd}(s)');
title('2-DOF PID: tracking vs disturbance response');

% In Simulink:
%  - Use separate summing blocks for b*r - y, r - y, and c*r - y.
%  - Implement Kp, Ki/s, Kd*s as gain, integrator, and derivative blocks.
%  - Connect plant P(s) using a Transfer Fcn block or a more detailed robot model.
