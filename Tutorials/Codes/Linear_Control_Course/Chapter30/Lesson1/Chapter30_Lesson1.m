% Parameters
omega_n = 1.0;
zeta    = 0.2;
K       = 8.0;
u_max   = 2.0;

% Nonlinear closed-loop dynamics
f = @(t, x) [ ...
    x(2); ...
    -2*zeta*omega_n*x(2) - omega_n^2*x(1) + ...
      omega_n^2*sat_fun(K*(1 - x(1)), u_max) ...
];

function u = sat_fun(v, u_max)
    u = min(max(v, -u_max), u_max);
end

% Simulate
tspan = [0 20];
x0 = [0; 0];
[t, x] = ode45(f, tspan, x0);
y = x(:, 1);

figure;
plot(t, y, 'LineWidth', 1.5);
grid on;
xlabel('t [s]');
ylabel('y(t)');
title('Closed-loop response with actuator saturation');
