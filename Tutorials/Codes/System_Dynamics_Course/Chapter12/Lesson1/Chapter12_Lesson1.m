% Chapter12_Lesson1.m
% System Dynamics (Control Engineering) — Chapter 12, Lesson 1
% Sinusoidal Steady-State Response and Frequency Response Definition
%
% Requires: Control System Toolbox (tf, bode, freqresp, lsim)
% Simulink section is optional and requires Simulink.
%
% This script:
% 1) Computes G(jw) and predicts steady-state amplitude/phase.
% 2) Simulates y(t) via lsim and estimates amplitude/phase by least squares.
% 3) (Optional) builds and simulates a Simulink model programmatically.

clear; clc;

wn   = 5.0;          % rad/s
zeta = 0.2;
G = tf([wn^2],[1 2*zeta*wn wn^2]);

Um    = 1.0;
omega = 4.0;         % rad/s
phi_u = 0.0;

% Frequency response at jw
Gjw = squeeze(freqresp(G, omega));   % complex scalar
Ym_pred = abs(Gjw) * Um;
phi_y_pred = angle(Gjw) + phi_u;

fprintf('G(jw) = %.6f%+.6fj\n', real(Gjw), imag(Gjw));
fprintf('|G(jw)| = %.6f, angle(G(jw)) = %.6f rad\n', abs(Gjw), angle(Gjw));
fprintf('Predicted steady-state amplitude Ym = %.6f\n', Ym_pred);
fprintf('Predicted steady-state phase phi_y = %.6f rad\n\n', phi_y_pred);

% Time simulation
t = linspace(0,40,40001).';
u = Um * sin(omega*t + phi_u);
y = lsim(G, u, t);

% Fit last 10 seconds: y ≈ a sin(wt) + b cos(wt)
mask = t >= (t(end)-10);
ts = t(mask); ys = y(mask);
A = [sin(omega*ts), cos(omega*ts)];
coef = A \ ys;
a = coef(1); b = coef(2);
Ym_hat = sqrt(a^2 + b^2);
phi_hat = atan2(b,a);

wrapToPi = @(x) mod(x + pi, 2*pi) - pi;

fprintf('Estimated from simulation (last 10 s):\n');
fprintf('Ym_hat = %.6f\n', Ym_hat);
fprintf('phi_y_hat = %.6f rad\n\n', wrapToPi(phi_hat));

fprintf('Errors:\n');
fprintf('Amplitude error: %.6f\n', Ym_hat - Ym_pred);
fprintf('Phase error (rad): %.6f\n', wrapToPi(phi_hat - phi_y_pred));

% Optional plots
% figure; plot(t,u,'LineWidth',1); hold on; plot(t,y,'LineWidth',1);
% grid on; xlabel('t (s)'); legend('u(t)','y(t)');
% xlim([t(end)-5 t(end)]);

% Optional: Build a Simulink model programmatically
%{
mdl = 'Chapter12_Lesson1_Simulink';
if bdIsLoaded(mdl); close_system(mdl,0); end
new_system(mdl); open_system(mdl);

add_block('simulink/Sources/Sine Wave',[mdl '/Sine']);
set_param([mdl '/Sine'], 'Amplitude', num2str(Um), 'Frequency', num2str(omega), 'Phase', num2str(phi_u));

add_block('simulink/Continuous/Transfer Fcn',[mdl '/G(s)']);
set_param([mdl '/G(s)'], 'Numerator', mat2str([wn^2]), 'Denominator', mat2str([1 2*zeta*wn wn^2]));

add_block('simulink/Sinks/Scope',[mdl '/Scope']);

add_line(mdl,'Sine/1','G(s)/1');
add_line(mdl,'G(s)/1','Scope/1');

set_param(mdl,'StopTime','40');
sim(mdl);

% The Scope shows the time response; to compare with theory, focus on late-time behavior.
%}
