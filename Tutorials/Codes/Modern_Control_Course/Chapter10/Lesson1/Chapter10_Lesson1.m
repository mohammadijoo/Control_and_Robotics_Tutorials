% Chapter10_Lesson1.m
%
% State steering for a continuous-time double integrator using a
% piecewise-constant input sequence.
%
% MATLAB/Simulink note:
% In Simulink, the same system can be represented by a State-Space block with
% A = [0 1; 0 0], B = [0; 1], C = eye(2), D = zeros(2,1), driven by a
% zero-order-hold signal containing the computed input samples u.

clear; clc; close all;

T = 2.0;
N = 60;
dt = T / N;

Phi = [1 dt; 0 1];
Gamma = [0.5 * dt^2; dt];

x0 = [0; 0];
xf = [1; 0];

S = zeros(2, N);
for k = 1:N
    S(:, k) = Phi^(N-k) * Gamma;
end

free_response = Phi^N * x0;
target_shift = xf - free_response;

rankS = rank(S);
if rankS < 2
    error("The finite-horizon steering map is rank deficient.");
end

u = S' * ((S*S') \ target_shift);

x = zeros(2, N+1);
x(:, 1) = x0;

for k = 1:N
    x(:, k+1) = Phi*x(:, k) + Gamma*u(k);
end

fprintf("Rank of finite-horizon steering map S: %d\n", rankS);
fprintf("Requested final state: [%g, %g]\n", xf(1), xf(2));
fprintf("Achieved final state:  [%g, %g]\n", x(1,end), x(2,end));
fprintf("Final steering error norm: %.6e\n", norm(x(:,end)-xf));
fprintf("Input energy approximation: %.12f\n", dt*sum(u.^2));

time = linspace(0, T, N+1);
input_time = linspace(0, T-dt, N);

figure;
plot(time, x(1,:), "LineWidth", 1.5); hold on;
plot(time, x(2,:), "LineWidth", 1.5);
grid on;
xlabel("time");
ylabel("state");
legend("position", "velocity");
title("State steering: double integrator");

figure;
stairs(input_time, u, "LineWidth", 1.5);
grid on;
xlabel("time");
ylabel("input u");
title("Minimum-norm piecewise-constant steering input");
