% ===== Code block 1 extracted from Chapter3/Lesson2.html =====
% Example: x_dot = A(t) x + b(t) with ode45
A_of_t = @(t) [0, 1; -2 - 0.2*sin(t), -0.4];
b_of_t = @(t) [0; 0.5*cos(t)];
f = @(t,x) A_of_t(t)*x + b_of_t(t);

tspan = [0, 10];
x0 = [1; 0];
opts = odeset('RelTol',1e-8,'AbsTol',1e-10);
[t,x] = ode45(f, tspan, x0, opts);

disp('Final x (ode45):');
disp(x(end,:).');
      

% ===== Code block 2 extracted from Chapter3/Lesson2.html =====
% Build a simple Simulink model for x_dot = A x + b (constant A, constant b)
A = [0 1; -2 -0.4];
b = [0; 0.5];

model = 'LinearVectorODE';
new_system(model); open_system(model);

add_block('simulink/Sources/Constant', [model '/b'], 'Value', 'b', 'Position', [30 80 80 110]);
add_block('simulink/Math Operations/Gain', [model '/A'], 'Gain', 'A', 'Multiplication', 'Matrix(K*u)', 'Position', [140 40 220 90]);
add_block('simulink/Math Operations/Sum', [model '/Sum'], 'Inputs', '++', 'Position', [270 55 295 85]);
add_block('simulink/Continuous/Integrator', [model '/Int'], 'Position', [340 40 370 90]);

% Initial condition
set_param([model '/Int'], 'InitialCondition', '[1;0]');

% Connections
add_line(model, 'Int/1', 'A/1');
add_line(model, 'A/1', 'Sum/1');
add_line(model, 'b/1', 'Sum/2');
add_line(model, 'Sum/1', 'Int/1');

set_param(model, 'StopTime', '10');
save_system(model);
      
