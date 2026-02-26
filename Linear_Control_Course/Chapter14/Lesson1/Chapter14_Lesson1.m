% First-order servo model: G(s) = K / (tau*s + 1)
K   = 10;
tau = 0.05;
s = tf('s');
G = K / (tau*s + 1);

% Frequency range [0.1, 1000] rad/s
w = logspace(-1, 3, 500);

% Compute frequency response
[mag, phase, wout] = bode(G, w);
mag = squeeze(mag);

% Convert magnitude to dB
mag_dB = 20*log10(mag);

figure;
semilogx(wout, mag_dB, 'LineWidth', 1.5);
grid on;
xlabel('Frequency \omega [rad/s]');
ylabel('Magnitude [dB]');
title('First-order servo magnitude response in dB');

% In robotics, G could be obtained from a robot joint model,
% e.g., using Robotics System Toolbox to derive linearized dynamics.
