% Chapter 12 - Lesson 2: Bode Plots: Magnitude, Phase, Asymptotes, and Construction Rules
% System Dynamics (Control Engineering)
%
% This script demonstrates:
%  1) MATLAB Control System Toolbox bode() for exact Bode plots.
%  2) Manual frequency response evaluation and asymptotic constructions.
%
% Requires: Control System Toolbox for tf/bode (manual parts run without it).

clear; clc; close all;

% Example transfer function:
%   G(s) = 10 * (1 + s/1) / ( s * (1 + s/10) )
K  = 10;
wz = 1;    % zero break (rad/s)
wp = 10;   % pole break (rad/s)

s = tf('s');
G = K * (1 + s/wz) / ( s * (1 + s/wp) );

w = logspace(-2, 3, 2000);  % rad/s

% ----- Exact Bode using toolbox -----
figure;
bode(G, w); grid on;
title('Exact Bode (MATLAB bode)');

% ----- Manual frequency response -----
jw = 1i*w;
Gj = K * (1 + jw/wz) ./ ( jw .* (1 + jw/wp) );

Mag = 20*log10(abs(Gj));
Ph  = unwrap(angle(Gj)) * 180/pi;

% ----- Asymptotic magnitude -----
MagAsym = 20*log10(K) ...
    + (w >= wz) .* (20*log10(w/wz)) ...
    - (w >= wp) .* (20*log10(w/wp)) ...
    - 20*log10(w);  % integrator 1/s

% ----- Asymptotic phase (standard approximation) -----
phaseZero = zeros(size(w));
phasePole = zeros(size(w));

% zero ramp from 0.1*wz to 10*wz
w1 = 0.1*wz; w2 = 10*wz;
mid = (w > w1) & (w < w2);
phaseZero(w >= w2) = 90;
phaseZero(mid) = 45*log10(w(mid)/w1);

% pole ramp from 0.1*wp to 10*wp
w1 = 0.1*wp; w2 = 10*wp;
mid = (w > w1) & (w < w2);
phasePole(w >= w2) = -90;
phasePole(mid) = -45*log10(w(mid)/w1);

PhAsym = phaseZero + phasePole - 90; % integrator contributes -90 deg

% ----- Plot comparison -----
figure;
semilogx(w, Mag, 'LineWidth', 1.2); hold on;
semilogx(w, MagAsym, '--', 'LineWidth', 1.2);
grid on; xlabel('w [rad/s]'); ylabel('Magnitude [dB]');
legend('Exact', 'Asymptotic'); title('Magnitude: exact vs asymptotic');

figure;
semilogx(w, Ph, 'LineWidth', 1.2); hold on;
semilogx(w, PhAsym, '--', 'LineWidth', 1.2);
grid on; xlabel('w [rad/s]'); ylabel('Phase [deg]');
legend('Exact', 'Asymptotic'); title('Phase: exact vs asymptotic');
