
A = [0 1; 0 -0.5];
B = [0; 1];
C = [1 0];
K = [3 2];
L = [5; 6];

Ts = 1e-3;
Tfinal = 5.0;
t = 0:Ts:Tfinal;
N = numel(t);

x = [0.5; 0.0];
x_hat = [0; 0];

q = zeros(1, N);
q_hat = zeros(1, N);

rng(0);

for k = 1:N
    y = C * x + 0.01 * randn;   % noisy measurement
    u = -K * x_hat;

    x_dot = A * x + B * u;
    x_hat_dot = A * x_hat + B * u + L * (y - C * x_hat);

    x = x + Ts * x_dot;
    x_hat = x_hat + Ts * x_hat_dot;

    q(k) = x(1);
    q_hat(k) = x_hat(1);
end

plot(t, q, t, q_hat, '--');
legend('q', 'q\_hat');
xlabel('time [s]');
ylabel('position [rad]');
title('Joint position with observer-based feedback');
