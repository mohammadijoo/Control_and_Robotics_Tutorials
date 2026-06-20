% Chapter26_Lesson1.m
%
% Need for steady-state accuracy in a state-space framework.
% Requires basic MATLAB. The optional ss/lsim part uses Control System Toolbox.

clear; clc; close all;

A = [0 1; -2 -3];
B = [0; 1];
C = [1 0];
D = 0;

K = [4 2];
Acl = A - B*K;

dcClosed = -C * (Acl \ B);
Nbar = 1 / dcClosed;

fprintf('Closed-loop DC map from prefilter input v to y: %.6f\n', dcClosed);
fprintf('Required static prefilter Nbar: %.6f\n\n', Nbar);

t = linspace(0, 8, 600);
x0 = [0; 0];

cases = {
    'Nbar = 1, no disturbance', 1.0, 1.0, 0.0;
    'Nbar = computed, no disturbance', Nbar, 1.0, 0.0;
    'Nbar = computed, disturbance d = 0.2', Nbar, 1.0, 0.2
};

figure; hold on; grid on;
for i = 1:size(cases, 1)
    label = cases{i, 1};
    nb = cases{i, 2};
    r = cases{i, 3};
    d = cases{i, 4};

    inputTotal = nb*r + d;

    % Since the input is constant:
    % x(t) = integral_0^t exp(Acl*(t-s))*B*inputTotal ds.
    % ode45 is used to keep the implementation transparent.
    f = @(time, x) Acl*x + B*inputTotal;
    [tout, xout] = ode45(f, t, x0);
    y = (C*xout')';

    fprintf('%s: final y approximately %.6f, final error %.6f\n', ...
            label, y(end), r - y(end));

    plot(tout, y, 'DisplayName', label);
end

yline(1.0, '--', 'reference', 'DisplayName', 'reference');
xlabel('time (s)');
ylabel('output y(t)');
title('Chapter 26 Lesson 1: steady-state accuracy');
legend('Location', 'best');

% Optional Control System Toolbox version:
% sys1 = ss(Acl, B, C, D);
% y1 = lsim(sys1, ones(size(t))*Nbar, t);
