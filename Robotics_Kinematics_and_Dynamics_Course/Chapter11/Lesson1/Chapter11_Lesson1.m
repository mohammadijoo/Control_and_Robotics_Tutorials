function qddot = pendulumEoM(t, x, tau, m, l, g)
% x = [q; qdot]
q    = x(1);
qdot = x(2);

M = m * l^2;
G = m * g * l * sin(q);
Cqdot = 0;   % no Coriolis term for 1-DOF

qddot = (tau - Cqdot - G) / M;
qddot = [qdot; qddot];
end

% Example numerical simulation with ODE45
m = 1.0; l = 1.0; g = 9.81;
tau_const = 0;

f = @(t, x) pendulumEoM(t, x, tau_const, m, l, g);
[t_sol, x_sol] = ode45(f, [0 10], [0.5; 0.0]);

plot(t_sol, x_sol(:,1)); xlabel('t'); ylabel('q(t)');
title('Pendulum angle using Lagrange-Euler EoM');
      
