% Parameters
a = 1.0;
b = 1.0;
K = 2.0;
h = 0.01;
N = 1000;
r = 1.0;

y_open   = zeros(1, N + 1);
y_closed = zeros(1, N + 1);

u_open = a / b * r;

for k = 1:N
    % Open-loop update
    y_open(k + 1) = y_open(k) + h * (-a * y_open(k) + b * u_open);

    % Closed-loop update
    e_k = r - y_closed(k);
    u_closed = K * e_k;
    y_closed(k + 1) = y_closed(k) + ...
        h * (-a * y_closed(k) + b * u_closed);
end

t = (0:N) * h;
plot(t, y_open, 'LineWidth', 1.5); hold on;
plot(t, y_closed, 'LineWidth', 1.5);
yline(r, '--', 'LineWidth', 1.0);
grid on;
xlabel('time [s]');
ylabel('output y(t)');
legend('open-loop', 'closed-loop', 'reference');
title('First-order plant: open vs closed loop');
