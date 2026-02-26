% Chapter1_Lesson1.m
% Autonomous Mobile Robots (Control Engineering) — Chapter 1, Lesson 1
% Lesson: What Makes Mobility Different from Manipulation
%
% This MATLAB script:
%   (i) simulates a differential-drive planar kinematic model, and
%   (ii) simulates a planar 2R manipulator end-effector trajectory,
% then (optionally) creates a simple Simulink model programmatically.
%
% Robotics libraries/toolboxes to explore next:
%   - Robotics System Toolbox (SE(2)/SE(3), rigidBodyTree, etc.)
%   - Simulink for model-based design and control prototyping
%
% Output:
%   Chapter1_Lesson1_mobile.csv
%   Chapter1_Lesson1_manipulator.csv
%   Chapter1_Lesson1_Simulink.slx  (if Simulink available)

clear; clc;

dt = 0.02;
T  = 8.0;
steps = floor(T/dt);

%% Mobile base: x_dot = v cos(theta), y_dot = v sin(theta), theta_dot = w
v = 0.5;    % m/s
w = 0.35;   % rad/s

x = 0; y = 0; th = 0;
mobile = zeros(steps+1, 4); % [t x y theta]
for k = 0:steps
    t = k*dt;
    mobile(k+1,:) = [t x y th];

    x  = x  + dt * v * cos(th);
    y  = y  + dt * v * sin(th);
    th = th + dt * w;
end
writematrix([["t","x","y","theta"]; string(mobile)], "Chapter1_Lesson1_mobile.csv");

%% 2R manipulator: end-effector (xe,ye) and joint angles (q1,q2)
q1 = 0.2; q2 = 0.9;
qdot1 = 0.25; qdot2 = -0.15;
l1 = 1.0; l2 = 0.7;

manip = zeros(steps+1, 5); % [t q1 q2 xe ye]
for k = 0:steps
    t = k*dt;
    xe = l1*cos(q1) + l2*cos(q1+q2);
    ye = l1*sin(q1) + l2*sin(q1+q2);
    manip(k+1,:) = [t q1 q2 xe ye];

    q1 = q1 + dt*qdot1;
    q2 = q2 + dt*qdot2;
end
writematrix([["t","q1","q2","xe","ye"]; string(manip)], "Chapter1_Lesson1_manipulator.csv");

%% Plots
figure; plot(mobile(:,2), mobile(:,3)); axis equal; grid on;
title("Mobile base trajectory (x-y)"); xlabel("x [m]"); ylabel("y [m]");

figure; plot(manip(:,4), manip(:,5)); axis equal; grid on;
title("2R manipulator end-effector trajectory (x-y)"); xlabel("x_e [m]"); ylabel("y_e [m]");

%% (Optional) Build a tiny Simulink model: integrate x_dot, y_dot, theta_dot
% This demonstrates the modeling difference: mobile base evolves on SE(2) with
% trigonometric coupling; manipulator typically uses joint integrators.
try
    mdl = "Chapter1_Lesson1_Simulink";
    if bdIsLoaded(mdl), close_system(mdl, 0); end
    new_system(mdl); open_system(mdl);

    % Blocks
    add_block("simulink/Sources/Constant", mdl + "/v", "Value", "0.5");
    add_block("simulink/Sources/Constant", mdl + "/w", "Value", "0.35");
    add_block("simulink/Continuous/Integrator", mdl + "/Int_theta", "InitialCondition", "0");
    add_block("simulink/Math Operations/Trigonometric Function", mdl + "/cos", "Operator", "cos");
    add_block("simulink/Math Operations/Trigonometric Function", mdl + "/sin", "Operator", "sin");
    add_block("simulink/Math Operations/Product", mdl + "/v_cos");
    add_block("simulink/Math Operations/Product", mdl + "/v_sin");
    add_block("simulink/Continuous/Integrator", mdl + "/Int_x", "InitialCondition", "0");
    add_block("simulink/Continuous/Integrator", mdl + "/Int_y", "InitialCondition", "0");
    add_block("simulink/Sinks/To Workspace", mdl + "/ToW", "VariableName", "simout", "SaveFormat", "StructureWithTime");

    % Lines
    add_line(mdl, "w/1", "Int_theta/1");
    add_line(mdl, "Int_theta/1", "cos/1");
    add_line(mdl, "Int_theta/1", "sin/1");
    add_line(mdl, "v/1", "v_cos/1"); add_line(mdl, "cos/1", "v_cos/2");
    add_line(mdl, "v/1", "v_sin/1"); add_line(mdl, "sin/1", "v_sin/2");
    add_line(mdl, "v_cos/1", "Int_x/1");
    add_line(mdl, "v_sin/1", "Int_y/1");

    % Log outputs (x,y,theta) via Mux-like struct: send as separate signals to To Workspace
    add_line(mdl, "Int_x/1", "ToW/1", "autorouting", "on");
    add_line(mdl, "Int_y/1", "ToW/2", "autorouting", "on");
    add_line(mdl, "Int_theta/1", "ToW/3", "autorouting", "on");

    set_param(mdl, "StopTime", num2str(T));
    save_system(mdl, mdl + ".slx");
    % Users can run: sim(mdl)
    fprintf("Created Simulink model: %s.slx\n", mdl);
catch ME
    fprintf("Simulink model build skipped (Simulink not available or error): %s\n", ME.message);
end

disp("Wrote: Chapter1_Lesson1_mobile.csv and Chapter1_Lesson1_manipulator.csv");
