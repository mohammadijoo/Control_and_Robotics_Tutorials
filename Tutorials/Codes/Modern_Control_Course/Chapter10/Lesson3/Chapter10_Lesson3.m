% Chapter10_Lesson3.m
% Finite-time reachability and minimum-energy steering in MATLAB/Simulink.
% Uses Control System Toolbox for ctrb; the core Gramian computation is explicit.

clear; clc;

A = [0 1; 0 0];
B = [0; 1];
x0 = [0; 0];
xT = [1; 0];

Cmat = ctrb(A, B);
disp("Controllability matrix:");
disp(Cmat);
disp("rank = " + rank(Cmat));

for T = [0.5 1.0 2.0 4.0]
    PhiT = expm(A*T);
    W = integral(@(s) gramianIntegrand(A, B, s), 0, T, ...
                 "ArrayValued", true, "RelTol", 1e-10, "AbsTol", 1e-12);
    W = 0.5*(W + W');
    d = xT - PhiT*x0;

    if norm(W*pinv(W)*d - d) > 1e-8
        error("Requested terminal state is not reachable over this horizon.");
    end

    Winv = pinv(W);
    energy = d' * Winv * d;

    u = @(t) B' * expm(A'*(T-t)) * Winv * d;

    fprintf("\nT = %.2f\n", T);
    disp("W(T) = "); disp(W);
    fprintf("det(W) = %.12g\n", det(W));
    fprintf("minimum energy = %.12g\n", energy);
    fprintf("u(0) = %.12g, u(T/2) = %.12g, u(T) = %.12g\n", ...
            u(0), u(T/2), u(T));
end

% Simulink note:
% Build a State-Space block with A=[0 1;0 0], B=[0;1], C=eye(2), D=zeros(2,1).
% Drive it with a MATLAB Function block implementing u(t)=B'*expm(A'*(T-t))*pinv(W)*d.
% Use a Clock block as the function input t and stop the simulation at t=T.

function G = gramianIntegrand(A, B, s)
    E = expm(A*s);
    G = E*B*B'*E';
end
