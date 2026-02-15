Ts = 0.01;        % Sampling period
J  = 0.02;        % Inertia
b  = 0.05;        % Viscous friction

A = [0 1;
     0 -b/J];
B = [0;
     1/J];
C = [1 0];
D = 0;

% Zero-order hold discretization
sysc = ss(A, B, C, D);
sysd = c2d(sysc, Ts, 'zoh');
Ad = sysd.A;
Bd = sysd.B;

% PD gains (from simulation tuning)
Kp = 8.0;
Kd = 1.5;

% State-space with PD on position and velocity (assuming full state feedback)
K  = [Kp Kd];
Acl = Ad - Bd * K;

% Simulate step response
N  = 1000;
x  = [0; 0];      % Initial state
xHist = zeros(2, N);
for k = 1:N
    xHist(:, k) = x;
    u = -K * x;   % reference = 0 for simplicity
    x = Ad * x + Bd * u;
end

t = (0:N-1) * Ts;
q = xHist(1, :);
plot(t, q); grid on;
xlabel('time [s]');
ylabel('q [rad]');
title('Discrete-time closed-loop response');
      
