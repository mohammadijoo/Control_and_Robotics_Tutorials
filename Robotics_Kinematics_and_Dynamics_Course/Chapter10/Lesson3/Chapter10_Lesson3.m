function dx = pendulum_rhs(t, x, params)
% x = [q; qd]
q  = x(1);
qd = x(2);

m = params.m;
l = params.l;
g = params.g;

% Example torque: PD about q = 0
Kp = 5.0;
Kd = 1.0;
tau = -Kp * q - Kd * qd;

qdd = (tau - m * g * l * sin(q)) / (m * l^2);

dx = [qd; qdd];
end

% Example of numerical integration:
params.m = 1.0;
params.l = 1.0;
params.g = 9.81;

x0 = [0.2; 0.0];  % initial angle and velocity
tspan = [0 10];

[t_sol, x_sol] = ode45(@(t, x) pendulum_rhs(t, x, params), tspan, x0);
      
