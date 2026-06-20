% Parameters
m = 1.0;
b = 0.4;
k = 4.0;

% ODE: m*ydd + b*yd + k*y = F(t), with F(t) = 1
msd = @(t, z) [ ...
    z(2);                           % z(1) = y, z(2) = dy/dt
    (1.0 - b * z(2) - k * z(1)) / m % acceleration
];

tspan = [0 10];
z0 = [0; 0];  % initial displacement and velocity

[t, z] = ode45(msd, tspan, z0);

y = z(:, 1);

figure;
plot(t, y, 'LineWidth', 1.5);
xlabel('t [s]');
ylabel('y(t) [m]');
title('Mass-spring-damper response (MATLAB)');
grid on;
