% ===== Code block 1 extracted from Chapter7/Lesson1.html =====
A = [0 1; -2 -3];
t0 = 0;
x0 = [1; 0];

ts = linspace(0,5,6);
X = zeros(2, numel(ts));

for i = 1:numel(ts)
    t = ts(i);
    X(:,i) = expm(A*(t-t0)) * x0;
end

disp('Times:'); disp(ts);
disp('States:'); disp(X);

% Cross-check by ODE solver (same equation xdot = A x)
f = @(t,x) A*x;
[t_ode, x_ode] = ode45(f, [0 5], x0);

% Optional: compare final state
x_expm_final = expm(A*(5-t0)) * x0;
x_ode_final = x_ode(end,:)';
disp('Final state (expm):'); disp(x_expm_final);
disp('Final state (ode45):'); disp(x_ode_final);
      

% ===== Code block 2 extracted from Chapter7/Lesson1.html =====
% Programmatic Simulink build for xdot = A x (u(t)=0)
A = [0 1; -2 -3];
B = [0; 0];
C = eye(2);
D = [0; 0];
x0 = [1; 0];

mdl = 'homogeneous_lti_expm_demo';
new_system(mdl); open_system(mdl);

add_block('simulink/Sources/Constant', [mdl '/u0'], 'Value', '0');
add_block('simulink/Continuous/State-Space', [mdl '/SS'], ...
    'A', 'A', 'B', 'B', 'C', 'C', 'D', 'D', 'X0', 'x0');
add_block('simulink/Sinks/To Workspace', [mdl '/x_out'], 'VariableName', 'x_sim');

set_param([mdl '/u0'], 'Position', [30 40 60 70]);
set_param([mdl '/SS'], 'Position', [120 30 260 90]);
set_param([mdl '/x_out'], 'Position', [320 40 390 70]);

add_line(mdl, 'u0/1', 'SS/1');
add_line(mdl, 'SS/1', 'x_out/1');

set_param(mdl, 'StopTime', '5');
save_system(mdl);

sim(mdl);

% x_sim will contain the state trajectory (as configured by To Workspace)
      
