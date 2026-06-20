% Second-order plant P(s) = 1/(s+1)^2 and proportional controller C(s) = k
s = tf('s');
P = 1/(s + 1)^2;

k = 5;
C = k;
L = C * P;
S = feedback(1, L);
T = feedback(L, 1);

% Bode magnitude of S and T
w = logspace(-1, 3, 1000);
[magS, ~] = bode(S, w);
[magT, ~] = bode(T, w);
magS = squeeze(magS);
magT = squeeze(magT);

% Approximate Bode sensitivity integral
lnS = log(magS);
I = trapz(w, lnS);
fprintf('Approximate Bode integral of ln|S(jw)| = %g\n', I);

figure;
semilogx(w, 20*log10(magS), 'LineWidth', 1.5); hold on;
semilogx(w, 20*log10(magT), '--', 'LineWidth', 1.5);
yline(0, ':');
grid on;
xlabel('Frequency (rad/s)');
ylabel('Magnitude (dB)');
title('Waterbed Effect in S(jw) and T(jw)');
legend('S(jw)', 'T(jw)', 'Location', 'Best');

% Simulink note:
% In Simulink, you can create a block diagram with:
% - Transfer Fcn block for P(s)
% - Gain block for C(s)
% - Sum block for feedback
% Then use the "Bode Plot" or "Frequency Response Estimation" tools
% to visualize the same waterbed effect interactively.
