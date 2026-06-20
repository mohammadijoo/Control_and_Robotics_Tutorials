% Parameters
wn   = 10;    % natural frequency [rad/s]
zeta = 0.4;   % damping ratio

% Closed-loop 2nd-order transfer function T(s)
s = tf('s');
T = wn^2 / (s^2 + 2*zeta*wn*s + wn^2);

% Bode data (frequency response)
w = logspace(-1, 2, 2000);
[mag, phase, wout] = bode(T, w);
mag = squeeze(mag);    % N-by-1
phase = squeeze(phase);

% Resonant peak and frequency
[Mr_num, idxMr] = max(mag);
wr_num = wout(idxMr);

% -3 dB bandwidth
mag0   = mag(1);
target = mag0 / sqrt(2);
idxBw  = find(mag <= target, 1, 'first');
wb_num = wout(idxBw);

fprintf('Numeric Mr  = %.3f\n', Mr_num);
fprintf('Numeric wr  = %.3f rad/s\n', wr_num);
fprintf('Numeric wb  = %.3f rad/s\n', wb_num);

% Compare to analytic expressions (if valid)
if zeta < 1/sqrt(2)
    wr_analytic = wn * sqrt(1 - 2*zeta^2);
    Mr_analytic = 1 / (2*zeta*sqrt(1 - zeta^2));
    fprintf('Analytic wr = %.3f rad/s\n', wr_analytic);
    fprintf('Analytic Mr = %.3f\n',     Mr_analytic);
end

% Plot Bode magnitude
figure;
bodemag(T, {0.1, 100});
grid on;
title('Closed-loop 2nd-order magnitude');

% Simulink remark:
% A Simulink model could use a Transfer Fcn block with numerator [wn^2]
% and denominator [1, 2*zeta*wn, wn^2] to represent the closed-loop joint.
% Tools like "Linear Analysis" can compute bandwidth directly from the model.
