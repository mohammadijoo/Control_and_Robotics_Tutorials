% Robot joint parameters
J = 0.01;
B = 0.1;
K = 1.0;
Kp = 20.0;

s = tf('s');
P = K / (J*s^2 + B*s);
C = Kp;           % P controller
L = C * P;        % open loop
T = feedback(L, 1);  % closed loop

% Bode plot with margins
figure;
margin(L); grid on;
title('Bode plot with gain/phase margins for robot joint loop');

% Numeric margins
[gm, pm, w_pc, w_gc] = margin(L);
gm_db = 20*log10(gm);
fprintf('GM (linear) = %.2f, GM (dB) = %.2f\n', gm, gm_db);
fprintf('PM = %.2f deg\n', pm);
fprintf('w_pc = %.2f rad/s, w_gc = %.2f rad/s\n', w_pc, w_gc);

% Approximate delay margin
pm_rad = pm*pi/180;
tau_d = pm_rad / w_gc;
fprintf('Approximate delay margin tau_d ~ %.4f s\n', tau_d);

% Closed-loop step response
figure;
step(T);
grid on;
title('Closed-loop step response of robot joint');

% In a robotics workflow, P could be replaced by a linearized
% joint model from Robotics System Toolbox, while the margin
% analysis commands remain the same.
