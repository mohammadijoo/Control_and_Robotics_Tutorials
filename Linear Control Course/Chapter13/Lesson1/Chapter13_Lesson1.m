% Parameters for first-order LTI system
K = 5;
tau = 0.1;

s = tf('s');
G = K / (tau * s + 1);

% Sinusoidal input
U = 1;
omega = 10;          % rad/s
t_final = 2;
t = linspace(0, t_final, 2000);
u = U * sin(omega * t);

% Time response to sinusoidal input
[y, t_out] = lsim(G, u, t);

% Theoretical complex gain at frequency omega
Gjw = freqresp(G, omega);
mag = abs(Gjw);
phase = angle(Gjw);  % radians

fprintf('Amplitude ratio |G(jw)| = %g\n', mag);
fprintf('Phase shift arg(G(jw)) = %g rad\n', phase);

figure;
plot(t_out, u, 'LineWidth', 1.2); hold on;
plot(t_out, y, 'LineWidth', 1.2);
xlabel('t [s]');
ylabel('Signals');
legend('u(t)', 'y(t)');
grid on;
title('Sinusoidal Input Response of First-Order System');
