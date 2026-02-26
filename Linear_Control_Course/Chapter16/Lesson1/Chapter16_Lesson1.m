% Parameters
J  = 0.01;
b  = 0.1;
Kt = 0.01;
Kp = 5.0;

s = tf('s');
G = Kt / (J * s^2 + b * s);  % plant
C = Kp;                      % proportional controller
L = C * G;                   % open-loop transfer function

% Simple Nichols plot using Control System Toolbox
figure;
nichols(L);
grid on;
title('Nichols Plot of Robotic Joint Open Loop');

% Manual Nichols construction from Bode data
w = logspace(-1, 3, 400);
[mag, phase] = bode(L, w);   % mag, phase: 3D arrays

mag = squeeze(mag);
phase = squeeze(phase);

magDb = 20 * log10(mag);
phaseDeg = phase;            % already in degrees

figure;
plot(phaseDeg, magDb);
xlabel('Phase (deg)');
ylabel('Magnitude (dB)');
title('Nichols Plot (Constructed from Bode Data)');
grid on;
set(gca, 'XDir', 'reverse'); % conventional Nichols orientation
