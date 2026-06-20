% Chapter2_Lesson1.m
% Autonomous Mobile Robots (Control Engineering)
% Chapter 2, Lesson 1: Rolling Constraints and Instantaneous Motion
%
% This script:
% 1) Builds lateral no-slip constraint matrix A such that A*[vx;vy;omega]=0
% 2) Computes the ICR (body frame) for omega ~= 0
% 3) Computes wheel spin rates from rolling constraint phi_dot = (1/R) t^T v_point
%
% Toolboxes (optional but recommended):
% - Robotics System Toolbox (for frames, visualization, and later navigation stack integration)
% - Symbolic Math Toolbox (if you want symbolic derivations)
%
% Note on Simulink:
% The quantities here map directly to a Simulink block chain:
%   (vx,vy,omega) -> v_point (per wheel) -> dot with t,n -> constraints + phi_dot
% You can implement v_point with Gain/Sum blocks and a small subsystem that computes [-ry; rx].

clear; clc;

% -------------------------
% Wheel definition
% -------------------------
% Each wheel: [lx, ly, alpha, R]
b = 0.6;
wheels = [
    0.0, +b/2, 0.0, 0.1;
    0.0, -b/2, 0.0, 0.1
];

% Sample twist in BODY frame
vx = 0.5; vy = 0.0; omega = 0.8;
xi = [vx; vy; omega];

% -------------------------
% Build A
% -------------------------
A = zeros(size(wheels,1), 3);
for i=1:size(wheels,1)
    lx = wheels(i,1); ly = wheels(i,2); alpha = wheels(i,3);
    n = [-sin(alpha); cos(alpha)];
    c = (-ly*n(1) + lx*n(2));
    A(i,:) = [n(1), n(2), c];
end

disp('A ='); disp(A);
disp('A*xi ='); disp(A*xi);

% -------------------------
% ICR in BODY frame
% -------------------------
if abs(omega) &lt; 1e-12
    disp('omega ~ 0: ICR at infinity (pure translation).');
    pICR = [NaN; NaN];
else
    pICR = [-vy/omega; vx/omega];
    disp('ICR (body) ='); disp(pICR);
end

% -------------------------
% Wheel spin rates
% -------------------------
phi_dot = zeros(size(wheels,1),1);
for i=1:size(wheels,1)
    lx = wheels(i,1); ly = wheels(i,2); alpha = wheels(i,3); R = wheels(i,4);
    t = [cos(alpha); sin(alpha)];
    vpt = [vx; vy] + omega * [-ly; lx];     % v + omega k x r
    phi_dot(i) = (t' * vpt) / R;
end
disp('Wheel spin rates [rad/s] ='); disp(phi_dot);

% -------------------------
% Optional: closest feasible twist (least-norm projection)
% -------------------------
v_des = [0.5; 0.2; 0.6];
P = eye(3) - A' * ((A*A' + 1e-12*eye(size(A,1))) \ A);
xi_proj = P * v_des;

disp('Desired twist ='); disp(v_des');
disp('Projected feasible twist ='); disp(xi_proj');
disp('Constraint residual A*xi_proj ='); disp((A*xi_proj)');

