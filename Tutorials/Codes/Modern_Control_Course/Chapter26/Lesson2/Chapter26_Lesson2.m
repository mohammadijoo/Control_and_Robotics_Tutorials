% Chapter26_Lesson2.m
% Modern Control - Chapter 26, Lesson 2
% Augmenting the State with Integral of Tracking Error
% Requires Control System Toolbox for place, ss, step.
% Optional: Simulink for the programmatic model at the end.

clear; clc; close all;

% Plant: x_dot = A x + B u, y = C x + D u
A = [0 1; -2 -3];
B = [0; 1];
C = [1 0];
D = 0;

n = size(A, 1);
p = size(C, 1);

% Integral state q_dot = r - y = -C x - D u + r
Aa = [A, zeros(n, p); -C, zeros(p, p)];
Ba = [B; -D];
Br = [zeros(n, p); eye(p)];
Ca = [C, zeros(p, p)];
Da = zeros(p, p);

disp('Augmented A matrix Aa ='); disp(Aa);
disp('Augmented B matrix Ba ='); disp(Ba);

disp('rank ctrb(A,B) ='); disp(rank(ctrb(A, B)));
disp('rank ctrb(Aa,Ba) ='); disp(rank(ctrb(Aa, Ba)));
R0 = [A B; C D];
disp('rank Rosenbrock matrix at zero [A B; C D] ='); disp(rank(R0));
disp('required rank n+p ='); disp(n+p);

% Pole placement is introduced in Lesson 3, but here it verifies that the
% augmented model is ready for state-feedback servo design.
desired_poles = [-2 -3 -4];
Kaug = place(Aa, Ba, desired_poles);
Kx = Kaug(:, 1:n);
Ki = -Kaug(:, n+1:end);   % because q_dot = r - y and u = -Kx*x + Ki*q

disp('Kaug for u = -Kaug*[x; q] ='); disp(Kaug);
disp('Equivalent Kx ='); disp(Kx);
disp('Equivalent Ki in u = -Kx*x + Ki*q ='); disp(Ki);

Acl = Aa - Ba*Kaug;
syscl = ss(Acl, Br, Ca, Da);
figure;
step(syscl, 8);
grid on;
title('Integral-augmented state feedback: unit-step tracking');

% Programmatic Simulink implementation of the closed-loop augmented model.
% It uses a Step source feeding the reference input r into the closed-loop
% augmented state-space system: xa_dot = Acl xa + Br r, y = Ca xa.
if exist('simulink', 'file')
    mdl = 'Chapter26_Lesson2_Simulink';
    if bdIsLoaded(mdl)
        close_system(mdl, 0);
    end
    new_system(mdl);
    open_system(mdl);

    add_block('simulink/Sources/Step', [mdl '/Reference Step'], ...
        'Position', [40 80 90 110], 'Time', '0', 'Before', '0', 'After', '1');
    add_block('simulink/Continuous/State-Space', [mdl '/Augmented Closed Loop'], ...
        'Position', [160 65 330 125]);
    add_block('simulink/Sinks/Scope', [mdl '/Output Scope'], ...
        'Position', [400 75 460 115]);

    set_param([mdl '/Augmented Closed Loop'], ...
        'A', mat2str(Acl), ...
        'B', mat2str(Br), ...
        'C', mat2str(Ca), ...
        'D', mat2str(Da));

    add_line(mdl, 'Reference Step/1', 'Augmented Closed Loop/1');
    add_line(mdl, 'Augmented Closed Loop/1', 'Output Scope/1');
    set_param(mdl, 'StopTime', '8');
    save_system(mdl);
    disp(['Created Simulink model: ' mdl '.slx']);
end
