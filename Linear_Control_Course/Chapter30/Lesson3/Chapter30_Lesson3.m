% Parameters
k = 5.0;
D = 3.0;
u_max = 1.0;
x0 = 0.0;
t_final = 20.0;
dt = 0.001;

N = floor(t_final / dt);
t = linspace(0, t_final, N + 1);
x = zeros(1, N + 1);
u_cmd = zeros(1, N + 1);
u_applied = zeros(1, N + 1);

x(1) = x0;

for n = 1:N
    u_cmd(n) = -k * x(n);
    u_applied(n) = max(-u_max, min(u_cmd(n), u_max));
    x_dot = -x(n) + u_applied(n) + D;
    x(n + 1) = x(n) + dt * x_dot;
end

u_cmd(N + 1) = -k * x(N + 1);
u_applied(N + 1) = max(-u_max, min(u_cmd(N + 1), u_max));

figure;
subplot(2,1,1);
plot(t, x);
ylabel('y(t) = x(t)');
grid on;

subplot(2,1,2);
plot(t, u_applied, t, u_cmd, '--');
ylabel('u(t)');
xlabel('time [s]');
legend('u\_applied', 'u\_cmd (linear)');
grid on;
