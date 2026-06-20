% Chapter10_Lesson2.m
% Reachable States and Reachable Subspace for continuous-time LTI systems
%
% This script computes:
%   1) R = [B AB ... A^(n-1)B]
%   2) an orthonormal basis for the reachable subspace
%   3) projection of a target state onto that subspace
%   4) a simple state-space simulation
%   5) an optional Simulink model built programmatically

clear; clc; close all;

A = [ 0  1  0;
     -2 -3  0;
      0  0 -1 ];

B = [0; 1; 0];
C = eye(3);
D = zeros(3,1);

n = size(A,1);

% Algebraic reachable subspace.
R = [];
Ak = eye(n);
for k = 0:n-1
    R = [R, Ak*B]; %#ok<AGROW>
    Ak = Ak*A;
end

reachableDimension = rank(R);
Q = orth(R);

disp('Reachability matrix R = [B AB A^2B]:');
disp(R);
disp(['Dimension of reachable subspace = ', num2str(reachableDimension)]);
disp('Orthonormal basis Q for reachable subspace:');
disp(Q);

xDesired = [1; -0.5; 2];
xReachable = Q*(Q'*xDesired);
xUnreachable = xDesired - xReachable;

disp('Desired target:');
disp(xDesired);
disp('Reachable component:');
disp(xReachable);
disp('Unreachable component:');
disp(xUnreachable);

% Continuous-time state-space simulation with a smooth input.
sys = ss(A,B,C,D);
t = linspace(0,8,500);
u = sin(2*t) + 0.4*cos(5*t);
[y,t,x] = lsim(sys,u,t,zeros(n,1));

figure;
plot(t,x,'LineWidth',1.5);
grid on;
xlabel('Time (s)');
ylabel('States');
legend('x1','x2','x3');
title('State response: x3 remains unaffected by the actuator from zero initial condition');

% Optional Simulink model generation.
% This creates a simple State-Space block driven by a Sine Wave block.
modelName = 'Chapter10_Lesson2_Simulink';
if bdIsLoaded(modelName)
    close_system(modelName,0);
end
new_system(modelName);
open_system(modelName);

add_block('simulink/Sources/Sine Wave',[modelName '/Input u']);
add_block('simulink/Continuous/State-Space',[modelName '/LTI System']);
add_block('simulink/Sinks/Scope',[modelName '/Scope']);

set_param([modelName '/LTI System'], ...
    'A','[0 1 0; -2 -3 0; 0 0 -1]', ...
    'B','[0; 1; 0]', ...
    'C','eye(3)', ...
    'D','zeros(3,1)');

add_line(modelName,'Input u/1','LTI System/1');
add_line(modelName,'LTI System/1','Scope/1');

set_param(modelName,'StopTime','8');
save_system(modelName);
disp(['Created Simulink model: ', modelName, '.slx']);
