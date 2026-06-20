
% Parameters
a = -1.0;
b = 1.0;

% ODE right-hand side
f = @(t, x) a * x + b * 1.0;   % u(t) = 1

tspan = [0 5];
x0 = 0.0;

[t, x] = ode45(f, tspan, x0);

% Analytical solution for comparison
x_analytical = 1 - exp(a * t);

% Plot
figure;
plot(t, x, 'o', t, x_analytical, '-');
xlabel('t');
ylabel('x(t)');
legend('Numerical', 'Analytical');
grid on;
      