s = tf('s');
G = 1 / (s * (s + 1));     % plant
Kp = 4; Ki = 3;
C = Kp + Ki / s;           % PI controller

L = C * G;
S = 1 / (1 + L);
T = L / (1 + L);

w = logspace(-2, 2, 400);
[magS, ~, ~] = bode(S, w);
[magT, ~, ~] = bode(T, w);

magS = squeeze(magS);
magT = squeeze(magT);
Ms = max(magS);
Mt = max(magT);
fprintf('M_S = %g, M_T = %g\n', Ms, Mt);

figure;
bodemag(S, T, w);
legend('S', 'T');
grid on;
title('Sensitivity and Complementary Sensitivity');

% In Simulink:
% - Build a unity-feedback loop with Transfer Fcn blocks for G(s) and C(s).
% - Use "Linear Analysis Tool" or "linmod" to obtain the linear model.
% - Use bode, bodemag, and sigma (for MIMO later) to inspect S and T.
