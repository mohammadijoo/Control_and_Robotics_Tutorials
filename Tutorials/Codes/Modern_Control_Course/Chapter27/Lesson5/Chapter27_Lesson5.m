% Chapter27_Lesson5.m
%
% Reference tracking and disturbance rejection in state space.
% Requires Control System Toolbox for place(). The simulation uses ode45.

clear; clc; close all;

% Mass-spring-damper plant:
% m*qddot + b*qdot + k*q = u + d, y = q
m = 1.0;
b = 0.6;
k = 2.0;

A = [0 1; -k/m -b/m];
B = [0; 1/m];
E = B;
C = [1 0];

r = 1.0;
d = 0.4;

% Case 1: state feedback plus feedforward prefilter.
poles_ff = [-2 -3];
K = place(A, B, poles_ff);
Nbar = -1 / (C * ((A - B*K) \ B));

% Case 2: integral servo.
Aaug = [A zeros(2,1); -C 0];
Baug = [B; 0];
poles_int = [-2 -3 -5];
Kaug = place(Aaug, Baug, poles_int);
Kx = Kaug(1:2);
Ki = -Kaug(3); % u = -Kx*x + Ki*z

fprintf('Feedforward K = [%g %g], Nbar = %g\n', K(1), K(2), Nbar);
fprintf('Integral Kx = [%g %g], Ki = %g\n', Kx(1), Kx(2), Ki);

% Feedforward simulations.
ff_rhs_0 = @(t,x) (A - B*K)*x + B*Nbar*r + E*0;
ff_rhs_d = @(t,x) (A - B*K)*x + B*Nbar*r + E*d;

[t0, x0] = ode45(ff_rhs_0, [0 8], [0;0]);
[t1, x1] = ode45(ff_rhs_d, [0 8], [0;0]);

y0 = (C*x0')';
y1 = (C*x1')';

% Integral simulation.
int_rhs = @(t,xa) [
    A*xa(1:2) + B*(-Kx*xa(1:2) + Ki*xa(3)) + E*d;
    r - C*xa(1:2)
];

[t2, xa2] = ode45(int_rhs, [0 8], [0;0;0]);
y2 = (C*xa2(:,1:2)')';
u2 = -xa2(:,1:2)*Kx' + Ki*xa2(:,3);

figure;
plot(t0, y0, 'LineWidth', 1.5); hold on;
plot(t1, y1, 'LineWidth', 1.5);
plot(t2, y2, 'LineWidth', 1.5);
yline(r, '--');
grid on;
xlabel('time [s]');
ylabel('output y');
title('Reference Tracking and Disturbance Rejection');
legend('FF, d=0', 'FF, d=0.4', 'Integral, d=0.4', 'reference', 'Location', 'best');

figure;
plot(t2, u2, 'LineWidth', 1.5);
grid on;
xlabel('time [s]');
ylabel('control input u');
title('Integral Controller Effort');

fprintf('Final y, feedforward no disturbance: %.8f\n', y0(end));
fprintf('Final y, feedforward with disturbance: %.8f\n', y1(end));
fprintf('Final y, integral with disturbance: %.8f\n', y2(end));

% Simulink note:
% Implement A,B,C,D in a State-Space block for the plant.
% Add a Gain block for -Kx, an Integrator block for z_dot = r-y,
% a Gain block for Ki, and a Sum block computing u = -Kx*x + Ki*z.
