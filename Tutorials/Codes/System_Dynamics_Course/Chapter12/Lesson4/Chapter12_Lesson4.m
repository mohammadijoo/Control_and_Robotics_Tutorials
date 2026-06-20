% Chapter12_Lesson4.m
% Nyquist and Nichols Plots (Introductory Level) — Frequency Response and Resonance
%
% Requirements:
%   - Control System Toolbox (for tf, nyquist, nichols, margin)
% Optional:
%   - Simulink (for the programmatic Simulink model section)

clear; clc; close all;

% Example open-loop plant:
% G(s) = K * wn^2 / (s^2 + 2*zeta*wn*s + wn^2) * 1/(tau*s + 1)
K = 5.0;
wn = 10.0;
zeta = 0.20;
tau = 0.05;

s = tf('s');
G = K * (wn^2)/(s^2 + 2*zeta*wn*s + wn^2) * 1/(tau*s + 1);

% Frequency grid
w = logspace(-1, 2.5, 1500);

% Nyquist plot
figure;
nyquist(G, w);
grid on;
title('Nyquist Curve');
saveas(gcf, 'Chapter12_Lesson4_nyquist_matlab.png');

% Nichols plot
figure;
nichols(G, w);
grid on;
title('Nichols Curve');
saveas(gcf, 'Chapter12_Lesson4_nichols_matlab.png');

% Margins
[Gm, Pm, Wcg, Wcp] = margin(G);
disp(['Gain margin (abs) = ', num2str(Gm)]);
disp(['Phase margin (deg) = ', num2str(Pm)]);
disp(['Gain crossover Wcg (rad/s) = ', num2str(Wcg)]);
disp(['Phase crossover Wcp (rad/s) = ', num2str(Wcp)]);

% Export frequency response to CSV (magnitude/phase)
[mag, ph] = bode(G, w);
mag = squeeze(mag);
ph = squeeze(ph);
mag_db = 20*log10(mag);

T = table(w(:), mag_db(:), ph(:), 'VariableNames', {'omega_rad_s','Mag_dB','Phase_deg'});
writetable(T, 'Chapter12_Lesson4_freqresp_matlab.csv');

disp('Wrote Chapter12_Lesson4_freqresp_matlab.csv');

%% Optional: programmatic Simulink model (creates a unity-feedback loop with Transfer Fcn)
%{
mdl = 'Chapter12_Lesson4_Simulink';
if bdIsLoaded(mdl); close_system(mdl, 0); end
new_system(mdl); open_system(mdl);

add_block('simulink/Continuous/Transfer Fcn', [mdl '/G']);
add_block('simulink/Math Operations/Sum', [mdl '/Sum']);
add_block('simulink/Sinks/Scope', [mdl '/Scope']);
add_block('simulink/Sources/Step', [mdl '/Step']);

set_param([mdl '/Sum'], 'Inputs', '+-');

% Transfer function coefficients for G(s)
num = K*wn^2;
den = conv([1 2*zeta*wn wn^2], [tau 1]);
set_param([mdl '/G'], 'Numerator', mat2str(num), 'Denominator', mat2str(den));

add_line(mdl, 'Step/1', 'Sum/1');
add_line(mdl, 'Sum/1', 'G/1');
add_line(mdl, 'G/1', 'Scope/1');
add_line(mdl, 'G/1', 'Sum/2');

set_param(mdl, 'StopTime', '5');
save_system(mdl);
%}
