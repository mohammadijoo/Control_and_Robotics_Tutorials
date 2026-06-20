
N = 100;
n = 0:N-1;

u = sin(0.1 * n);   % example input

% Static linear system: y[k] = 2 * u[k]
y_static_linear = 2 * u;

% Static nonlinear system: y[k] = u[k]^2
y_static_nonlinear = u.^2;

% Dynamic linear system: y[k+1] = a*y[k] + b*u[k]
a = 0.8;
b = 0.2;
y_dynamic = zeros(size(u));
for k = 1:(N-1)
    y_dynamic(k+1) = a * y_dynamic(k) + b * u(k);
end

% Plot to visualize dynamic memory
figure;
plot(n, u, 'DisplayName', 'u[k]');
hold on;
plot(n, y_dynamic, 'DisplayName', 'y[k]');
xlabel('k');
ylabel('Amplitude');
legend('show');
grid on;
      