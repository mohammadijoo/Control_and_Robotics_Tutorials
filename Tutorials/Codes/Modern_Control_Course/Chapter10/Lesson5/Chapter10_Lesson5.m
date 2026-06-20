% Chapter10_Lesson5.m
% Modern Control - Chapter 10, Lesson 5
% Examples of controllable and uncontrollable systems.
%
% MATLAB libraries:
%   Control System Toolbox: ctrb, ss, rank, eig.
%   Simulink connection: the ss(A,B,C,D) objects below can be used with
%   State-Space blocks or lsim for time-domain checks.

clear; clc;

function analyzeSystem(name, A, B)
    n = size(A, 1);
    Ctr = ctrb(A, B);
    r = rank(Ctr);
    fprintf('\n%s\n', name);
    fprintf('%s\n', repmat('-', 1, strlength(name)));
    disp('Controllability matrix:');
    disp(Ctr);
    fprintf('rank(C) = %d out of n = %d\n', r, n);
    if r == n
        disp('Conclusion: controllable');
    else
        disp('Conclusion: uncontrollable');
    end
end

% Example 1: double integrator.
A1 = [0 1;
      0 0];
B1 = [0; 1];
analyzeSystem("Example 1: double integrator", A1, B1);

% Example 2: two decoupled masses, one actuator.
A2 = [0 1 0 0;
      0 0 0 0;
      0 0 0 1;
      0 0 0 0];
B2 = [0; 1; 0; 0];
analyzeSystem("Example 2: two decoupled masses, one actuator", A2, B2);

% Example 3: coupled two-mass oscillator.
m1 = 1.0; m2 = 1.0;
k1 = 1.0; k2 = 1.2; kc = 0.8;
c1 = 0.1; c2 = 0.2;

A3 = [0, 1, 0, 0;
     -(k1+kc)/m1, -c1/m1, kc/m1, 0;
      0, 0, 0, 1;
      kc/m2, 0, -(k2+kc)/m2, -c2/m2];
B3 = [0; 1/m1; 0; 0];
analyzeSystem("Example 3: coupled two-mass oscillator", A3, B3);

% Example 4: diagonal modal systems.
A4 = diag([-1, -2, -3]);
B4a = [1; 1; 1];
B4b = [1; 0; 1];

analyzeSystem("Example 4a: diagonal system, all modes actuated", A4, B4a);
analyzeSystem("Example 4b: diagonal system, middle mode not actuated", A4, B4b);

% State-space model object for simulation or Simulink State-Space block data.
C3 = eye(4);
D3 = zeros(4, 1);
sys3 = ss(A3, B3, C3, D3);

% Simple time response for Example 3 with a sinusoidal input.
t = linspace(0, 15, 1000);
u = sin(t);
[y, tOut, x] = lsim(sys3, u, t);

figure;
plot(tOut, x);
grid on;
xlabel('time (s)');
ylabel('states');
title('Chapter10 Lesson5: State response of coupled two-mass system');
legend('q1', 'v1', 'q2', 'v2');
