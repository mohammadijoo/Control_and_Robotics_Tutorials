% Parameters
J = 0.01;   % kg m^2
b = 0.1;    % N m s/rad
R_a = 1.0;  % ohm
L_a = 0.5;  % H
K_t = 0.01; % N m/A
K_e = 0.01; % V s/rad

% ODE for [theta; omega; i_a]
motor_ode = @(t, x) [ ...
    x(2); ...
    (K_t * x(3) - b * x(2)) / J; ...
    (24.0 - R_a * x(3) - K_e * x(2)) / L_a ...
];

[t, x] = ode45(motor_ode, [0 2], [0; 0; 0]);

theta = x(:, 1);
omega = x(:, 2);
i_a   = x(:, 3);

figure;
subplot(3, 1, 1); plot(t, theta); ylabel('\theta (rad)');
subplot(3, 1, 2); plot(t, omega); ylabel('\omega (rad/s)');
subplot(3, 1, 3); plot(t, i_a);   ylabel('i_a (A)'); xlabel('t (s)');
