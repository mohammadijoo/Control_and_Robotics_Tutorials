% Chapter12_Lesson3.m
% System Dynamics — Chapter 12, Lesson 3
% Resonance, Bandwidth, and Quality Factor in Mechanical and Electrical Systems
%
% Toolboxes:
%   - Control System Toolbox (tf, bode)
%   - Simulink (optional section builds a small model programmatically)

clear; clc;

%% 1) Mass–spring–damper example: m xdd + c xd + k x = f
m = 1.0;     % kg
c = 0.4;     % N*s/m
k = 100.0;   % N/m

wn   = sqrt(k/m);
zeta = c/(2*sqrt(k*m));
Q    = 1/(2*zeta);

fprintf("Mass–spring–damper:\n");
fprintf("  wn   = %.6g rad/s\n", wn);
fprintf("  zeta = %.6g\n", zeta);
fprintf("  Q    = %.6g\n", Q);

% Resonant frequency exists only if zeta < 1/sqrt(2)
if zeta < 1/sqrt(2)
    wr = wn*sqrt(1 - 2*zeta^2);
    fprintf("  wr   = %.6g rad/s\n", wr);

    % Half-power frequencies relative to peak (exact)
    r1sq = 1 - 2*zeta^2 - 2*zeta*sqrt(1 - zeta^2);
    r2sq = 1 - 2*zeta^2 + 2*zeta*sqrt(1 - zeta^2);
    w1 = wn*sqrt(r1sq);
    w2 = wn*sqrt(r2sq);
    BW = w2 - w1;
    fprintf("  w1   = %.6g rad/s\n", w1);
    fprintf("  w2   = %.6g rad/s\n", w2);
    fprintf("  BW   = %.6g rad/s\n", BW);
    fprintf("  Q_hp = %.6g (wr/BW)\n", wr/BW);
    fprintf("  BW approx (2 zeta wn) = %.6g rad/s\n", 2*zeta*wn);
else
    fprintf("  wr   = (no resonant peak; zeta >= 1/sqrt(2))\n");
end

%% 2) Frequency response check using transfer functions
% Force -> displacement: G(s) = 1/(m s^2 + c s + k)
sys = tf(1, [m c k]);

w = logspace(-1, 3, 4000);  % rad/s
[mag, phase] = bode(sys, w);
mag = squeeze(mag);

% Estimate peak and half-power points numerically
[Mr, idx] = max(mag);
wr_hat = w(idx);
target = Mr / sqrt(2);

% left crossing
w1_hat = NaN;
for i = idx:-1:2
    if (mag(i)-target)*(mag(i-1)-target) <= 0
        t = (target - mag(i-1))/(mag(i) - mag(i-1));
        w1_hat = w(i-1) + t*(w(i) - w(i-1));
        break;
    end
end

% right crossing
w2_hat = NaN;
for i = idx:1:(numel(w)-1)
    if (mag(i)-target)*(mag(i+1)-target) <= 0
        t = (target - mag(i))/(mag(i+1) - mag(i));
        w2_hat = w(i) + t*(w(i+1) - w(i));
        break;
    end
end

fprintf("\nEstimated from bode samples:\n");
fprintf("  wr_hat  = %.6g rad/s\n", wr_hat);
fprintf("  Mr_hat  = %.6g\n", Mr);
fprintf("  w1_hat  = %.6g rad/s\n", w1_hat);
fprintf("  w2_hat  = %.6g rad/s\n", w2_hat);
fprintf("  BW_hat  = %.6g rad/s\n", w2_hat - w1_hat);
fprintf("  Q_hat   = %.6g\n", wr_hat/(w2_hat - w1_hat));

%% 3) Series RLC quick computation (theoretical)
R = 10.0; L = 50e-3; C = 10e-6;
w0 = 1/sqrt(L*C);
Q_rlc = w0*L/R;
BW_rlc = R/L;
fprintf("\nSeries RLC:\n");
fprintf("  w0 = %.6g rad/s\n", w0);
fprintf("  Q  = %.6g\n", Q_rlc);
fprintf("  BW = %.6g rad/s\n", BW_rlc);

%% 4) Optional: Build a small Simulink model programmatically
% The model: Sine Wave -> Transfer Fcn (1/(m s^2 + c s + k)) -> Scope
% This is a minimal setup that can be used for frequency sweeps.

build_simulink = true;
if build_simulink
    mdl = "Chapter12_Lesson3_Simulink";
    if bdIsLoaded(mdl)
        close_system(mdl, 0);
    end
    new_system(mdl); open_system(mdl);

    add_block("simulink/Sources/Sine Wave", mdl + "/Sine");
    add_block("simulink/Continuous/Transfer Fcn", mdl + "/Plant");
    add_block("simulink/Sinks/Scope", mdl + "/Scope");

    set_param(mdl + "/Plant", "Numerator", "1", "Denominator", sprintf("[%g %g %g]", m, c, k));

    add_line(mdl, "Sine/1", "Plant/1");
    add_line(mdl, "Plant/1", "Scope/1");

    set_param(mdl, "StopTime", "10");
    save_system(mdl);
    fprintf("\nCreated Simulink model: %s\n", mdl);
end
