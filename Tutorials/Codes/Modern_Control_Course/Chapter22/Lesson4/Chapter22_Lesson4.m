% Chapter22_Lesson4.m
% Requirements for Implementing State Feedback
%
% This script checks direct state availability, controllability,
% PBH stabilizability, closed-loop eigenvalues, and optionally creates
% a minimal Simulink model for x_dot = (A - B*K)x.
%
% Toolboxes:
%   - Base MATLAB is enough for the custom controllability/observability code.
%   - Control System Toolbox provides ctrb/obsv/ss/initial conveniences.
%   - Simulink is optional for model construction.

clear; clc;

A = [0 1; -2 -3];
B = [0; 1];

% Position-only measurement cannot directly implement u = -Kx.
C_position_only = [1 0];

% Full-state measurement or reconstructed full state.
C_full = eye(2);

% Gain K is assumed to be supplied by a design step.
K = [4 2];

n = size(A,1);

Wc = local_ctrb(A,B);
Wo_position = local_obsv(A,C_position_only);
Wo_full = local_obsv(A,C_full);

fprintf("=== Requirement checks for u = -Kx + r ===\n");
fprintf("rank(Wc) = %d, required = %d\n", rank(Wc), n);
fprintf("rank(C_position_only) = %d, required for direct full-state measurement = %d\n", ...
    rank(C_position_only), n);
fprintf("rank(C_full) = %d, required for direct full-state measurement = %d\n", ...
    rank(C_full), n);

fprintf("rank(Wo_position) = %d, required for observer-based reconstruction = %d\n", ...
    rank(Wo_position), n);
fprintf("rank(Wo_full) = %d, required for observer-based reconstruction = %d\n", ...
    rank(Wo_full), n);

[pbh_ok, bad_modes] = local_pbh_stabilizable(A,B);
fprintf("PBH stabilizable? %d\n", pbh_ok);
if ~isempty(bad_modes)
    disp("Unstabilizable nondecaying modes:");
    disp(bad_modes);
end

Acl = A - B*K;
disp("Closed-loop matrix Acl = A - BK:");
disp(Acl);
disp("Closed-loop eigenvalues:");
disp(eig(Acl));

x0 = [1; 0];
[t,x] = ode45(@(t,x) Acl*x, [0 5], x0);
fprintf("Final simulated state at t = %.2f: [%g, %g]^T\n", t(end), x(end,1), x(end,2));

% Optional: create a minimal Simulink model if Simulink is available.
if exist("simulink", "file") == 4
    model = "Chapter22_Lesson4_Simulink";
    if bdIsLoaded(model)
        close_system(model,0);
    end
    new_system(model);
    open_system(model);

    assignin("base","Acl",Acl);
    assignin("base","Bzero",zeros(2,1));
    assignin("base","Cout",eye(2));
    assignin("base","Dzero",zeros(2,1));

    add_block("simulink/Sources/Constant", model + "/ZeroInput", ...
        "Value", "0", "Position", [80 90 130 120]);
    add_block("simulink/Continuous/State-Space", model + "/ClosedLoopPlant", ...
        "A", "Acl", "B", "Bzero", "C", "Cout", "D", "Dzero", ...
        "X0", "[1;0]", "Position", [200 70 360 140]);
    add_block("simulink/Sinks/Scope", model + "/StateScope", ...
        "Position", [440 75 500 135]);
    add_line(model, "ZeroInput/1", "ClosedLoopPlant/1");
    add_line(model, "ClosedLoopPlant/1", "StateScope/1");
    save_system(model);
    fprintf("Created optional Simulink model: %s.slx\n", model);
end

function Wc = local_ctrb(A,B)
    n = size(A,1);
    Wc = [];
    Ak = eye(n);
    for k = 1:n
        Wc = [Wc, Ak*B]; %#ok<AGROW>
        Ak = Ak*A;
    end
end

function Wo = local_obsv(A,C)
    n = size(A,1);
    Wo = [];
    Ak = eye(n);
    for k = 1:n
        Wo = [Wo; C*Ak]; %#ok<AGROW>
        Ak = Ak*A;
    end
end

function [ok,bad_modes] = local_pbh_stabilizable(A,B)
    n = size(A,1);
    ev = eig(A);
    bad_modes = [];
    tol = 1e-9;
    for k = 1:length(ev)
        lam = ev(k);
        if real(lam) >= -tol
            M = [lam*eye(n)-A, B];
            if rank(M) < n
                bad_modes = [bad_modes; lam]; %#ok<AGROW>
            end
        end
    end
    ok = isempty(bad_modes);
end
