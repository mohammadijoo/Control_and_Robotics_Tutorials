% Chapter9_Lesson1.m
% Modern Control — Chapter 9, Lesson 1
% Stability, Asymptotic Stability, and Instability (State-Space View)
%
% Demonstrates continuous-time LTI stability using eig(A), expm(A*t),
% Lyapunov equation checks, and a programmatic Simulink state-space model.

clear; clc; close all;

A_stable = [-1  2;
            -3 -2];

A_center = [ 0  1;
            -1  0];

A_unstable = [0.2  1;
              0   -1];

x0 = [1; -0.5];

reportSystem("Stable spiral", A_stable, x0);
reportSystem("Marginal center", A_center, x0);
reportSystem("Unstable saddle/source component", A_unstable, x0);

% Plot exact trajectories using the matrix exponential.
t = linspace(0, 10, 500);
Xstable = zeros(2, numel(t));
Xcenter = zeros(2, numel(t));
Xunstable = zeros(2, numel(t));

for k = 1:numel(t)
    Xstable(:, k) = expm(A_stable * t(k)) * x0;
    Xcenter(:, k) = expm(A_center * t(k)) * x0;
    Xunstable(:, k) = expm(A_unstable * t(k)) * x0;
end

figure;
plot(t, vecnorm(Xstable), "LineWidth", 1.5); hold on;
plot(t, vecnorm(Xcenter), "LineWidth", 1.5);
plot(t, vecnorm(Xunstable), "LineWidth", 1.5);
grid on;
xlabel("time t");
ylabel("state norm ||x(t)||");
legend("stable", "center", "unstable", "Location", "best");
title("State norm behavior for stability classes");

% Simulink-oriented construction of x_dot = A x with output y = x.
% This section requires Simulink. It creates a model with a State-Space block.
model = "Chapter9_Lesson1_StateSpace_Model";
if license("test", "Simulink")
    if bdIsLoaded(model)
        close_system(model, 0);
    end
    new_system(model);
    open_system(model);

    add_block("simulink/Continuous/State-Space", model + "/State-Space");
    set_param(model + "/State-Space", ...
        "A", mat2str(A_stable), ...
        "B", mat2str(zeros(2, 1)), ...
        "C", mat2str(eye(2)), ...
        "D", mat2str(zeros(2, 1)), ...
        "X0", mat2str(x0));

    add_block("simulink/Sources/Constant", model + "/ZeroInput");
    set_param(model + "/ZeroInput", "Value", "0");

    add_block("simulink/Sinks/Scope", model + "/Scope");

    add_line(model, "ZeroInput/1", "State-Space/1");
    add_line(model, "State-Space/1", "Scope/1");

    set_param(model, "StopTime", "10");
    save_system(model);
    disp("Simulink model created and saved: " + model + ".slx");
else
    disp("Simulink license not available; skipped model creation.");
end

function reportSystem(name, A, x0)
    fprintf("\n=== %s ===\n", name);
    disp("A ="); disp(A);

    lambda = eig(A);
    disp("eigenvalues ="); disp(lambda);

    alpha = max(real(lambda));
    tol = 1e-10;

    if alpha < -tol
        classification = "asymptotically stable";
    elseif alpha > tol
        classification = "unstable";
    else
        classification = "marginal-candidate: inspect Jordan structure";
    end

    disp("classification = " + classification);

    x10 = expm(A * 10) * x0;
    fprintf("||x(0)|| = %.6f\n", norm(x0));
    fprintf("||x(10)|| = %.6f\n", norm(x10));

    if classification == "asymptotically stable"
        P = lyap(A', eye(size(A)));
        disp("P solving A'P + P A = -I:");
        disp(P);
        disp("eig(P) =");
        disp(eig(P));
    end
end
