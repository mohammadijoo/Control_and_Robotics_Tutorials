% Chapter21_Lesson3.m
% Zero Dynamics and Internal System Behavior
%
% Requires Control System Toolbox for ss, tf, zero/tzero, and lsim.
% Simulink note:
%   Use a State-Space block with A,B,C,D below.
%   Feed u = 18*x1 by exposing the state x1 or by using an equivalent
%   MATLAB Function block implementing the zero-output feedback.

clear; clc; close all;

A = [0 1 0;
     0 0 1;
    -6 -11 -6];

B = [0; 0; 1];
C = [-1 1 0];
D = 0;

sys = ss(A,B,C,D);
disp('Transfer function:')
G = tf(sys)

disp('Transmission zero(s):')
try
    z = tzero(sys)
catch
    z = zero(sys)
end

disp('Poles:')
p = pole(sys)

% Output-nulling manifold:
% y = -x1 + x2 = 0  => x2 = x1
% ydot = -x2 + x3 = 0 => x3 = x1
% Enforce d/dt(x3 - x1)=0 => u = 18*x1
eta0 = 0.02;
x0 = [eta0; eta0; eta0];

odefun = @(t,x) A*x + B*(18*x(1));
tspan = linspace(0,5,501);
[t,x] = ode45(odefun, tspan, x0);

u = 18*x(:,1);
y = (C*x.' + D*u.').';

fprintf('max |y(t)| = %.3e\n', max(abs(y)));
fprintf('eta(5) numerical = %.9f\n', x(end,1));
fprintf('eta(5) exact     = %.9f\n', eta0*exp(5));

figure;
plot(t, x(:,1), 'LineWidth', 1.5);
grid on;
xlabel('time');
ylabel('eta = x_1');
title('Zero Dynamics: eta_dot = eta');

figure;
plot(t, y, 'LineWidth', 1.5);
grid on;
xlabel('time');
ylabel('y');
title('Output Held at Zero');
