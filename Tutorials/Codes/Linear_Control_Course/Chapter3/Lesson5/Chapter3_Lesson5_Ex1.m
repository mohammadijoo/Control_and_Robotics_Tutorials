% Define linear analysis points
io(1) = linio('pendulum_model/u',     1, 'input');
io(2) = linio('pendulum_model/theta', 1, 'output');

% Compute an operating point (e.g. steady-state)
op = operpoint('pendulum_model');

% Linearize about operating point
sys_lin = linearize('pendulum_model', op, io);

A = sys_lin.A;
B = sys_lin.B;
C = sys_lin.C;
D = sys_lin.D;
