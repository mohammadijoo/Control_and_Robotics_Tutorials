% Chapter 12 - Lesson 5: Time–Frequency Domain Relationships and Trade-offs
% File: Chapter12_Lesson5.m
%
% This MATLAB script links time-domain step metrics (rise time, overshoot, settling time)
% to frequency-domain metrics (bandwidth, resonant peak) for the canonical 2nd-order low-pass:
%   G(s) = wn^2 / (s^2 + 2*zeta*wn*s + wn^2)
%
% Required toolbox:
%   - Control System Toolbox (tf, stepinfo, bode, bandwidth)
%
% Optional:
%   - Simulink (scripted model construction section)

clear; clc;

zeta = 0.35;
wn   = 12; % rad/s

s = tf('s');
G = wn^2/(s^2 + 2*zeta*wn*s + wn^2);

% Time-domain metrics
info = stepinfo(G); % default: 2% settling, 10-90% rise
Mp = info.Overshoot;
tr = info.RiseTime;
ts = info.SettlingTime;

% Frequency-domain metrics
[mag,~,w] = bode(G);
mag = squeeze(mag);
w   = squeeze(w);

Mr = max(mag);
wr = w(find(mag == Mr, 1, 'first'));
wb = bandwidth(G); % -3 dB bandwidth

fprintf('=== MATLAB demo: 2nd-order time-frequency tradeoffs ===\n');
fprintf('zeta=%.3f, wn=%.3f rad/s\n', zeta, wn);
fprintf('Overshoot Mp = %.2f %%\n', Mp);
fprintf('Rise time tr  = %.6f s\n', tr);
fprintf('Settling ts   = %.6f s\n', ts);
fprintf('Resonant peak Mr = %.6f\n', Mr);
fprintf('Resonant freq wr = %.6f rad/s\n', wr);
fprintf('Bandwidth wb     = %.6f rad/s\n', wb);

% Sweep zeta
zetas = [0.15 0.25 0.35 0.50 0.70];
fprintf('\n=== Sweep zeta (fixed wn) ===\n');
fprintf('zeta, Mp(%%), Mr, wb(rad/s)\n');
for z = zetas
    Gz = wn^2/(s^2 + 2*z*wn*s + wn^2);
    infz = stepinfo(Gz);
    Mpz = infz.Overshoot;
    [magz,~,wz] = bode(Gz);
    magz = squeeze(magz); wz = squeeze(wz);
    Mrz = max(magz);
    wbz = bandwidth(Gz);
    fprintf('%.2f, %.2f, %.4f, %.4f\n', z, Mpz, Mrz, wbz);
end

% Optional: scripted Simulink model creation (requires Simulink)
try
    mdl = 'Chapter12_Lesson5_Simulink';
    if bdIsLoaded(mdl); close_system(mdl, 0); end
    new_system(mdl); open_system(mdl);

    add_block('simulink/Sources/Step', [mdl '/Step'], 'Position',[50 80 80 110]);
    add_block('simulink/Continuous/Transfer Fcn', [mdl '/G(s)'], 'Position',[150 75 260 115]);
    add_block('simulink/Sinks/Scope', [mdl '/Scope'], 'Position',[320 75 350 115]);

    set_param([mdl '/G(s)'], 'Numerator', mat2str([wn^2]), ...
                             'Denominator', mat2str([1 2*zeta*wn wn^2]));

    add_line(mdl, 'Step/1', 'G(s)/1');
    add_line(mdl, 'G(s)/1', 'Scope/1');

    set_param(mdl, 'StopTime', '3');
    sim(mdl);

    % save_system(mdl, [mdl '.slx']);  % optional
catch ME
    fprintf('\nSimulink section skipped: %s\n', ME.message);
end
