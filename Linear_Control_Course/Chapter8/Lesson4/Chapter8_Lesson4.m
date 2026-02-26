% Plant and PI controller
s = tf('s');
G = 5 / (s + 3);          % first-order plant
Kp = 1;
Ki = 4;
C = Kp + Ki / s;          % PI controller

L = C * G;
T = feedback(L, 1);       % closed-loop from r to y
S = 1 - T;                % sensitivity from r to e

% Static error constants
Kp_static = dcgain(L);      % position constant
Kv_static = dcgain(s * L);  % velocity constant

fprintf('Kp (static) = %g\n', Kp_static);
fprintf('Kv (static) = %g\n', Kv_static);
fprintf('Predicted ramp error (unit ramp) = %g\n', 1 / Kv_static);

% Step response (zero steady-state error)
figure;
step(T);
title('Step response with PI controller');

% Ramp response via lsim
t = 0:0.01:10;
r_ramp = t;   % unit-slope ramp
[y_ramp, t_out] = lsim(T, r_ramp, t);

figure;
plot(t_out, r_ramp, 'LineWidth', 1.5); hold on;
plot(t_out, y_ramp, 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Position');
legend('r(t) = ramp', 'y(t)');
title('Ramp tracking with PI controller');

% Simulink notes:
% A corresponding Simulink model uses:
%  - Sum block (r - y)
%  - Discrete or continuous PI controller block
%  - Transfer Fcn block for G(s)
%  - Scope blocks to view y(t) and e(t)
