% Physical parameters
m = 1.0;
c = 0.4;
k = 4.0;

% State-space matrices
A = [0,      1;
    -k/m, -c/m];
B = [0;
     1/m];
C = [1, 0];
D = 0;

% Zero-input internal dynamics: u(t) = 0
u_fun = @(t) 0;

% State ODE for ode45: x_dot = A x + B u
f = @(t, x) A * x + B * u_fun(t);

tspan = [0, 10];
x0 = [1; 0];  % y(0) = 1, y_dot(0) = 0

[t_sol, x_sol] = ode45(f, tspan, x0);

y_sol = (C * x_sol.').';  % output y(t) = x1(t)

% Plot
% figure;
% plot(t_sol, y_sol);
% xlabel('t'); ylabel('y(t)');
% title('Mass-Spring-Damper Zero-Input Internal Dynamics');

% Simulink (conceptual):
% - Create a State-Space block with A, B, C, D as above.
% - Set the input to zero using a Constant block with value 0.
% - Scope the output to visualize y(t).
      
