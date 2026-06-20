% Chapter28_Lesson4.m
% Qualitative effect of different Q and R weights on closed-loop behavior.
% Requires: Control System Toolbox for lqr, ss, and initial.
% Simulink extension:
%   Use a State-Space block with Acl = A - B*K for each case, or build
%   the feedback loop with State-Space Plant, Gain K, Sum, and Scope blocks.

clear; clc; close all;

A = [0 1; -1 -0.15];
B = [0; 1];
C = eye(2);
D = zeros(2,1);
x0 = [1; 0];

cases = {
    'balanced',             diag([1 1]),       1;
    'high_position_weight', diag([25 1]),      1;
    'high_velocity_weight', diag([1 25]),      1;
    'high_input_penalty',   diag([1 1]),       25;
    'low_input_penalty',    diag([1 1]),       0.04
};

t = linspace(0, 8, 900);

figure; hold on; grid on;
title('Changing Q and R changes the redesigned closed-loop transient');
xlabel('time [s]');
ylabel('state x_1');

fprintf('\n%-24s %-24s %-28s %-12s\n', 'case', 'K', 'eig(A-BK)', 'max|u|');
fprintf('%s\n', repmat('-', 1, 92));

for i = 1:size(cases,1)
    name = cases{i,1};
    Q = cases{i,2};
    R = cases{i,3};

    K = lqr(A, B, Q, R);
    Acl = A - B*K;
    sys_cl = ss(Acl, B, C, D);

    [y, t_out, x] = initial(sys_cl, x0, t);
    u = -(K*x')';

    plot(t_out, y(:,1), 'DisplayName', name);

    lam = eig(Acl);
    fprintf('%-24s [%.4f %.4f]      [% .4f%+.4fi, % .4f%+.4fi]   %.4f\n', ...
        name, K(1), K(2), ...
        real(lam(1)), imag(lam(1)), ...
        real(lam(2)), imag(lam(2)), ...
        max(abs(u)));
end

legend('Location','best');

% In Simulink:
% 1. Put the plant in a State-Space block: x_dot = A x + B u, y = x.
% 2. Feed y into a Gain block K.
% 3. Use a Sum block to implement u = -K x.
% 4. Repeat for each Q,R case and compare Scope outputs.
