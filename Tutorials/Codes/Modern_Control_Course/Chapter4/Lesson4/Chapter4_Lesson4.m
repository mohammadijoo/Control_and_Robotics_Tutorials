% Physical parameters
m = 1.0; b = 0.4; k = 4.0;

% State choice: x1 = q, x2 = qdot
A = [0 1; -k/m -b/m];
B = [0; 1/m];
C = [1 0];
D = 0;

sys = ss(A,B,C,D);

% Step response (force input)
t = linspace(0,10,2001);
u = ones(size(t)); % step
[y,t,x] = lsim(sys,u,t,[0;0]);

disp("Final state x(T):");
disp(x(end,:));
disp("Final output y(T):");
disp(y(end));

% --- Programmatic Simulink build (integrator-chain form) ---
% This constructs xdot = A x + B u with Integrator blocks for states.
model = "msd_statespace_demo";
new_system(model); open_system(model);

add_block("simulink/Sources/Step", model + "/u");
add_block("simulink/Math Operations/Gain", model + "/B_gain");
set_param(model + "/B_gain", "Gain", "1/m");

% Two integrators for x2 and x1 (x1dot = x2)
add_block("simulink/Continuous/Integrator", model + "/Int_x2");
add_block("simulink/Continuous/Integrator", model + "/Int_x1");

% Gains for A terms in x2dot: -(k/m)*x1 -(b/m)*x2 + (1/m)*u
add_block("simulink/Math Operations/Gain", model + "/Gain_k");
set_param(model + "/Gain_k", "Gain", "-k/m");
add_block("simulink/Math Operations/Gain", model + "/Gain_b");
set_param(model + "/Gain_b", "Gain", "-b/m");

add_block("simulink/Math Operations/Sum", model + "/Sum_x2dot");
set_param(model + "/Sum_x2dot", "Inputs", "+++");

% Wire: x1 = Int_x1 output, x2 = Int_x2 output
% x1dot = x2
add_line(model, "Int_x2/1", "Int_x1/1");

% x2dot sum: Gain_k*x1 + Gain_b*x2 + B_gain*u
add_line(model, "Int_x1/1", "Gain_k/1");
add_line(model, "Gain_k/1", "Sum_x2dot/1");

add_line(model, "Int_x2/1", "Gain_b/1");
add_line(model, "Gain_b/1", "Sum_x2dot/2");

add_line(model, "u/1", "B_gain/1");
add_line(model, "B_gain/1", "Sum_x2dot/3");

% Feed x2dot into Int_x2
add_line(model, "Sum_x2dot/1", "Int_x2/1");

% Output y = x1
add_block("simulink/Sinks/Scope", model + "/Scope_y");
add_line(model, "Int_x1/1", "Scope_y/1");

set_param(model, "StopTime", "10");
save_system(model);
      
