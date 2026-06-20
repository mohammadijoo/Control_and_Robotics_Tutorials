% Parameters
A = 1.0;
phi_u = deg2rad(30);   % rad
omega = 10.0;          % rad/s
tau = 0.05;            % s

% Input and output phasors
U = A * exp(1j * phi_u);      % U* = A e^{j phi}
H = 1 ./ (1 + 1j * tau * omega);
Y = H .* U;

magH = abs(H);
phaseH = angle(H);

fprintf("H(j omega) magnitude = %.4f\n", magH);
fprintf("H(j omega) phase [deg] = %.2f\n", rad2deg(phaseH));

% Time-domain reconstruction
t = linspace(0, 0.5, 2000);
u_t = real(U * exp(1j * omega * t));
y_t = real(Y * exp(1j * omega * t));

plot(t, u_t, t, y_t);
xlabel("t [s]"); ylabel("signal");
legend("u(t)", "y(t)");
grid on;
title("Sinusoidal steady-state via phasors");

% Simulink note:
% In Simulink, a Sine Wave block feeding a first-order transfer block (e.g. 1/(tau s + 1))
% will produce the same amplitude ratio and phase shift predicted by |H(j omega)| and angle(H).
