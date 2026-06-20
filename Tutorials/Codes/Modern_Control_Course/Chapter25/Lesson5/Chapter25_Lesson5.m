% Chapter25_Lesson5.m
% Structural Constraints: Limited Actuators and Sparse Feedback
%
% MATLAB / Simulink implementation.
% Recommended toolboxes:
%   Control System Toolbox: ctrb, place, lqr, ss, initial
%   Simulink: automatic block-diagram construction at the end of this file

clear; clc; close all;

A = [ 0.0   1.0   0.0   0.0;
     -2.0  -0.25  0.7   0.0;
      0.0   0.0   0.0   1.0;
      0.6   0.0  -1.5  -0.20 ];

B_candidates = [0.0  0.0  0.0;
                1.0  0.2  0.0;
                0.0  0.0  0.0;
                0.0  0.4  1.0];

n = size(A,1);
q = size(B_candidates,2);

fprintf("Actuator subset analysis\n");
fprintf("------------------------\n");
for maskInt = 1:(2^q - 1)
    idx = find(bitget(maskInt,1:q));
    Bsel = B_candidates(:,idx);
    Cmat = ctrb(A,Bsel);
    fprintf("subset = ");
    fprintf("%d ", idx);
    fprintf(", rank(C) = %d / %d\n", rank(Cmat), n);
end

selected = [1 3];
B = B_candidates(:,selected);

Q = diag([20 1 20 1]);
R = diag([0.5 0.5]);

% Dense LQR design.
Kdense = lqr(A,B,Q,R);

% Sparse feedback pattern:
% actuator 1 receives x1,x2; actuator 2 receives x3,x4.
Mask = [1 1 0 0;
        0 0 1 1];

KsparseProjection = Kdense .* Mask;

fprintf("\nDense LQR gain:\n");
disp(Kdense);
fprintf("Sparse projected gain:\n");
disp(KsparseProjection);

fprintf("Dense closed-loop eigenvalues:\n");
disp(eig(A - B*Kdense));
fprintf("Sparse projected closed-loop eigenvalues:\n");
disp(eig(A - B*KsparseProjection));

% Optional nonlinear optimization over only the allowed entries of K.
% This searches for a sparse K whose closed-loop polynomial approximates
% the desired pole polynomial.
desiredPoles = [-1.2+1.0i, -1.2-1.0i, -1.8+0.8i, -1.8-0.8i];
targetPoly = poly(desiredPoles);

allowed = find(Mask(:) == 1);
theta0 = Kdense(allowed);

objective = @(theta) sparsePoleObjective(theta, A, B, Mask, allowed, targetPoly);
thetaStar = fminsearch(objective, theta0);
Kopt = zeros(size(Mask));
Kopt(allowed) = thetaStar;

fprintf("Optimized sparse gain:\n");
disp(Kopt);
fprintf("Optimized sparse closed-loop eigenvalues:\n");
disp(eig(A - B*Kopt));

sysDense = ss(A - B*Kdense, eye(n), eye(n), zeros(n));
sysSparse = ss(A - B*Kopt, eye(n), eye(n), zeros(n));
x0 = [1; 0; -0.7; 0.2];

figure;
initial(sysDense, sysSparse, x0, 10);
grid on;
legend("dense LQR", "optimized sparse");

% Simulink model generation: x_dot = A x + B u, u = -Kopt x.
modelName = "Chapter25_Lesson5_Simulink";
if bdIsLoaded(modelName)
    close_system(modelName,0);
end
new_system(modelName);
open_system(modelName);

add_block("simulink/Sources/Constant", modelName + "/zero_input", ...
    "Value", "0", "Position", [40 80 90 110]);

add_block("simulink/Continuous/State-Space", modelName + "/closed_loop_ss", ...
    "A", "A - B*Kopt", ...
    "B", "zeros(4,1)", ...
    "C", "eye(4)", ...
    "D", "zeros(4,1)", ...
    "X0", "x0", ...
    "Position", [150 55 300 135]);

add_block("simulink/Sinks/Scope", modelName + "/states_scope", ...
    "Position", [370 65 430 125]);

add_line(modelName, "zero_input/1", "closed_loop_ss/1");
add_line(modelName, "closed_loop_ss/1", "states_scope/1");

set_param(modelName, "StopTime", "10");
save_system(modelName);

function J = sparsePoleObjective(theta, A, B, Mask, allowed, targetPoly)
    K = zeros(size(Mask));
    K(allowed) = theta;
    p = poly(eig(A - B*K));
    poleError = norm(real(p - targetPoly),2)^2;
    gainPenalty = 1.0e-3 * norm(K,"fro")^2;
    J = poleError + gainPenalty;
end
