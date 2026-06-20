% Chapter16_Lesson2.m
% Difference Equations and Discrete-Time State-Space Models

clear; clc; close all;

%% Example 1: Scalar first-order difference equation
% y(k+1) = a*y(k) + b*u(k)
N = 40;
u1 = ones(1, N);
a = 0.85;
b = 0.2;
y = zeros(1, N+1);
y(1) = 0.0; % y(0)

for k = 1:N
    y(k+1) = a*y(k) + b*u1(k);
end

disp('Scalar recursion (first 10 samples, y(k))');
for k = 1:10
    fprintf('k=%2d, y=% .6f\n', k-1, y(k));
end

figure;
stem(0:N, y, 'filled');
grid on;
xlabel('k');
ylabel('y[k]');
title('Scalar Difference Equation: y[k+1] = a y[k] + b u[k]');

%% Example 2: Companion-form state-space
% y(k+2) - 1.5 y(k+1) + 0.7 y(k) = u(k)
% Let x(k) = [y(k+1); y(k)]
A = [1.5, -0.7;
     1.0,  0.0];
B = [1.0;
     0.0];
C = [0.0, 1.0];
D = 0.0;

u2 = zeros(1, N);
u2(1:10) = 1.0;  % finite pulse

nx = size(A,1);
X = zeros(nx, N+1);
Y = zeros(1, N);
x = [0.0; 0.0];

X(:,1) = x;
for k = 1:N
    Y(k) = C*x + D*u2(k);
    x = A*x + B*u2(k);
    X(:,k+1) = x;
end

disp('State-space output (first 10 samples)');
for k = 1:10
    fprintf('k=%2d, y=% .6f, x=[% .6f, % .6f]\n', k-1, Y(k), X(1,k), X(2,k));
end

figure;
stem(0:N-1, Y, 'filled');
grid on;
xlabel('k');
ylabel('y[k]');
title('Discrete-Time State-Space Output (Companion Form)');

%% Optional toolbox check (if Control System Toolbox is available)
% sysd = ss(A, B, C, D, 1); % sample time Ts = 1
% [y_toolbox, t_toolbox, x_toolbox] = lsim(sysd, u2, 0:N-1, [0;0]);
