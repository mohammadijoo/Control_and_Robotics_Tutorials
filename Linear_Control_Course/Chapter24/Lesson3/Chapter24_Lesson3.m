% Robot joint parameters
J = 0.02;
B = 0.1;
K = 1.0;

s = tf('s');
P = K / (J * s^2 + B * s);

kp = 50;
Ti = 0.1;
Tl = 0.02;
alpha = 0.2;

C = kp * (1 + 1/(Ti*s)) * ((Tl*s + 1)/(alpha*Tl*s + 1));

L = C * P;
S = feedback(1, L);     % S = 1/(1+L)
T = feedback(L, 1);     % T = L/(1+L)

% Uncertainty weight
W2 = 0.2 * (s/50 + 1) / (s/500 + 1);

w = logspace(-1, 3, 600);

[magS, ~, ~] = bode(S, w);
MS = max(squeeze(magS));

[magWT, ~, ~] = bode(W2 * T, w);
robustIndex = max(squeeze(magWT));

fprintf('Maximum sensitivity M_S ≈ %.2f\n', MS);
fprintf('max |W2(jw) T(jw)| ≈ %.2f\n', robustIndex);

figure; bodemag(L, S, T, w); grid on;
legend('L','S','T');

figure; nichols(L, w); grid on;

% In Simulink, the same loop can be realized with Transfer Fcn blocks
% and PID Controller blocks, and robustness evaluated via linearization
% (linmod / linearize) around operating points of a Robotics System Toolbox
% manipulator model.
