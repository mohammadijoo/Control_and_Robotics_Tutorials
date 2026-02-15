% Plant and controllers
s = tf('s');
P = 1 / (s * (s + 1) * (s + 2));

% Conservative PI
Kp1 = 2; Ki1 = 1;
C1 = Kp1 + Ki1 / s;

% Aggressive PI
Kp2 = 6; Ki2 = 4;
C2 = Kp2 + Ki2 / s;

L1 = C1 * P;
L2 = C2 * P;

T1 = feedback(L1, 1);
T2 = feedback(L2, 1);

figure;
bode(L1, 'b', L2, 'r');
grid on;
legend('Conservative L1', 'Aggressive L2');
title('Loop-shape comparison');

figure;
step(T1, T2);
grid on;
legend('Conservative T1', 'Aggressive T2');
title('Step response comparison');

info1 = stepinfo(T1);
info2 = stepinfo(T2);
disp(info1);
disp(info2);

% Simulink integration:
% - Implement P(s) and C(s) using Transfer Fcn and Sum blocks.
% - Use 'Scope' blocks to monitor y(t) and u(t).
% - Add band-limited white noise at the sensor to visualize noise amplification.
