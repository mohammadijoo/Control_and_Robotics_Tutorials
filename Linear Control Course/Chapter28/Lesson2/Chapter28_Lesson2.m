% Parameters
omega_n = 20;    % rad/s
zeta    = 0.4;

% Transfer function G(s) = omega_n^2 / (s^2 + 2*zeta*omega_n*s + omega_n^2)
num = omega_n^2;
den = [1, 2*zeta*omega_n, omega_n^2];
G   = tf(num, den);

% 1) Time-domain responses
figure; step(G);
title('Step response');

figure; impulse(G);
title('Impulse response');

t = 0:0.001:4;
u = sin(5*t);   % sinusoidal input (e.g., joint torque disturbance)
[y, t_out] = lsim(G, u, t);

figure;
plot(t_out, u, 'k--', t_out, y, 'b');
legend('u(t)', 'y(t)');
xlabel('t [s]');
ylabel('Amplitude');
title('Forced response to sinusoidal input');
grid on;

% 2) Frequency response (Bode and Nyquist)
figure; bode(G);
grid on;

figure; nyquist(G);
grid on;

% 3) Robotics-oriented example:
%    Combine with Robotics System Toolbox to analyze joint servo dynamics
%    for a rigid-body model robot = robotics.RigidBodyTree;
%    Closed-loop joint model can be mapped to a transfer function and
%    analyzed with the same commands.
