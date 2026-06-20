% Chapter 13 - Vibrations and Multi-Degree-of-Freedom (MDOF) Systems
% Lesson 5 - Intro to Experimental Modal Analysis and System Identification Concepts
%
% File: Chapter13_Lesson5.m
%
% This script:
% 1) Builds a 3-DOF mass-spring-damper model.
% 2) Simulates broadband excitation with lsim.
% 3) Estimates FRF using H1 = G_yu / G_uu via Welch averages (pwelch/cpsd).
% 4) Computes coherence, does rough peak-picking + half-power damping.
% 5) (Optional) Auto-builds a simple Simulink model containing the same state-space block.

clear; clc; close all;

%% 1) Model (3-DOF chain)
m1=1.0; m2=0.9; m3=0.8;
M = diag([m1 m2 m3]);

k1=800; k2=600; k3=500; k4=700;
K = [k1+k2  -k2      0;
     -k2    k2+k3   -k3;
      0     -k3     k3+k4];

c1=2.0; c2=1.8; c3=1.5; c4=2.2;
C = [c1+c2  -c2      0;
     -c2    c2+c3   -c3;
      0     -c3     c3+c4];

n = 3;
Minv = inv(M);
Z = zeros(n); I = eye(n);

A = [Z I; -Minv*K -Minv*C];
B = [zeros(n); Minv];    % inputs are forces at each DOF

% output: acceleration at DOF1 (index 1) => qdd = -Minv*K q - Minv*C qd + Minv*f
Ca = [-Minv*K -Minv*C];
Da = Minv;

sysa = ss(A, B, Ca, Da);

%% 2) Excitation + simulation
fs = 500;                 % Hz
T  = 40;                  % seconds
t  = (0:1/fs:T-1/fs)';

rng(7);
u = randn(length(t),1);
[b,a] = butter(4, 0.25);  % normalized cutoff
u = filtfilt(b,a,u);
u = u/std(u);

F = zeros(length(t), n);
F(:,1) = u; % force on DOF1

y = lsim(sysa, F, t);     % y is 3 columns: acceleration at each DOF
y1 = y(:,1) + 0.02*randn(size(y(:,1)));

%% 3) FRF estimation (H1) and coherence
nperseg = 4096;
nover   = 2048;
win     = hanning(nperseg);

[Guu,f] = pwelch(u, win, nover, nperseg, fs, 'onesided');
[Gyy,~] = pwelch(y1, win, nover, nperseg, fs, 'onesided');
[Gyu,~] = cpsd(y1, u, win, nover, nperseg, fs, 'onesided');

H1 = Gyu ./ (Guu + 1e-30);
coh = (abs(Gyu).^2) ./ (Guu.*Gyy + 1e-30);

figure; semilogy(f, abs(H1)); xlim([0 120]);
xlabel('Frequency [Hz]'); ylabel('|H1(jw)|'); title('Estimated FRF (H1)');

figure; plot(f, coh); ylim([0 1.05]); xlim([0 120]);
xlabel('Frequency [Hz]'); ylabel('Coherence'); title('Magnitude-squared coherence');

%% 4) Peak-picking + half-power damping (rough)
fmax = 80;
mask = (f > 0.5) & (f < fmax) & (coh > 0.6);
ff = f(mask);
Hm = abs(H1(mask));

[pks, locs] = findpeaks(Hm, ff, 'MinPeakProminence', prctile(Hm,70), 'MinPeakDistance', 2);
locs = locs(1:min(3, numel(locs)));

fprintf('Rough modal estimates (fn Hz, zeta, f1,f2):\n');
for i = 1:numel(locs)
    fn = locs(i);
    peak = interp1(ff, Hm, fn);
    target = peak/sqrt(2);

    % left
    idx0 = find(ff <= fn, 1, 'last');
    idx1 = idx0;
    while idx1 > 2 && Hm(idx1) > target
        idx1 = idx1 - 1;
    end
    f1 = interp1([Hm(idx1) Hm(idx1+1)], [ff(idx1) ff(idx1+1)], target);

    % right
    idx2 = idx0;
    while idx2 < numel(ff)-1 && Hm(idx2) > target
        idx2 = idx2 + 1;
    end
    f2 = interp1([Hm(idx2-1) Hm(idx2)], [ff(idx2-1) ff(idx2)], target);

    zeta = (f2 - f1)/(2*fn);
    fprintf('  fn=%7.3f Hz, zeta=%7.4f, f1=%7.3f, f2=%7.3f\n', fn, zeta, f1, f2);
end

%% 5) Optional: build a simple Simulink model programmatically
% This makes a model with:
% [Band-Limited White Noise] -> [State-Space] -> [To Workspace]
%
% You can comment this section if you prefer manual modeling.

mdl = 'Chapter13_Lesson5_Simulink';
if bdIsLoaded(mdl); close_system(mdl,0); end
if exist([mdl '.slx'], 'file'); delete([mdl '.slx']); end

new_system(mdl);
open_system(mdl);

add_block('simulink/Sources/Band-Limited White Noise', [mdl '/Noise']);
set_param([mdl '/Noise'], 'SampleTime', num2str(1/fs), 'NoisePower', '1', 'Seed', '7');

add_block('simulink/Continuous/State-Space', [mdl '/Plant']);
set_param([mdl '/Plant'], 'A', 'A', 'B', 'B(:,1)', 'C', 'Ca(1,:)', 'D', 'Da(1,1)');

add_block('simulink/Sinks/To Workspace', [mdl '/y_to_ws']);
set_param([mdl '/y_to_ws'], 'VariableName', 'y_sim', 'SaveFormat', 'Array');

add_line(mdl, 'Noise/1', 'Plant/1');
add_line(mdl, 'Plant/1', 'y_to_ws/1');

set_param(mdl, 'StopTime', num2str(T));
save_system(mdl);

disp(['Simulink model saved: ' mdl '.slx']);
