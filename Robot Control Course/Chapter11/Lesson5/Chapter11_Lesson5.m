
% Model parameters
M = 2.0;
b = 0.8;
k = 5.0;

A = [0 1;
     -k/M -b/M];
B = [0;
     1/M];
C = [1 0];

K = [27.0 13.6];
L = [28.4;
     242.14];

dt = 1e-3;
T_end = 5.0;
N = floor(T_end / dt);

q_ref = 0.5;
x_ref = [q_ref; 0];

x = [0; 0];
x_hat = [0; 0];

t_log = zeros(N, 1);
q_log = zeros(N, 1);
q_hat_log = zeros(N, 1);
u_log = zeros(N, 1);

for k_step = 1:N
    t = (k_step - 1) * dt;
    t_log(k_step) = t;

    y = C * x;
    y_hat = C * x_hat;
    y_err = y - y_hat;

    u = -K * (x_hat - x_ref);

    q_log(k_step) = y;
    q_hat_log(k_step) = y_hat;
    u_log(k_step) = u;

    x_dot = A * x + B * u;
    x_hat_dot = A * x_hat + B * u + L * y_err;

    x = x + dt * x_dot;
    x_hat = x_hat + dt * x_hat_dot;
end

figure;
plot(t_log, q_log, t_log, q_hat_log, '--', ...
     t_log, q_ref * ones(size(t_log)), ':');
xlabel('Time [s]');
ylabel('Joint position [rad]');
legend('q (true)', 'q_hat (estimate)', 'q_{ref}');
grid on;

figure;
plot(t_log, u_log);
xlabel('Time [s]');
ylabel('Torque u [N*m]');
grid on;
