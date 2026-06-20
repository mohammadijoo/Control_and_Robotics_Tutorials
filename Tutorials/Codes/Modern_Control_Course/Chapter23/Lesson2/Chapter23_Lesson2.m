% Chapter23_Lesson2.m
% Pole Assignment via Controllable Canonical Form (CCF)
%
% Convention:
%   A_c has ones on the superdiagonal and last row [-a0 ... -a_{n-1}].
%   b_c = [0 ... 0 1]'.
%   u = -K_c z + r.
%   K_c = [alpha0-a0, alpha1-a1, ..., alpha_{n-1}-a_{n-1}].

clear; clc;

% Open-loop p(s)=s^3+6s^2+11s+6
a = [6 11 6];              % [a0 a1 a2]
desiredPoles = [-4 -5 -6];

[Ac, bc] = companion_pair_from_coefficients(a);
Kc = ccf_gain(a, desiredPoles);
Acl = Ac - bc*Kc;

disp('A_c ='); disp(Ac);
disp('b_c ='); disp(bc);
disp('K_c ='); disp(Kc);
disp('eig(A_c-b_c*K_c) ='); disp(eig(Acl).');

% Optional Control System Toolbox verification:
if exist('place', 'file') == 2
    K_place = place(Ac, bc, desiredPoles);
    disp('K from MATLAB place(Ac,bc,desiredPoles) =');
    disp(K_place);
end

% MATLAB simulation of the closed-loop state response
C = eye(size(Ac,1));
D = zeros(size(Ac,1),1);
sys_cl = ss(Acl, bc, C, D);
t = linspace(0, 5, 300);
r = ones(size(t));
[y, t, x] = lsim(sys_cl, r, t);

figure('Name','Chapter23 Lesson2 CCF Pole Assignment');
plot(t, x, 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('States');
title('Closed-loop response in controllable canonical coordinates');
legend('z_1','z_2','z_3');

% Optional Simulink model generation if Simulink is installed.
% This creates a State-Space block with the closed-loop matrix.
if exist('new_system', 'file') == 2
    model = 'Chapter23_Lesson2_Simulink_CCF';
    if bdIsLoaded(model)
        close_system(model, 0);
    end
    new_system(model);
    open_system(model);

    add_block('simulink/Sources/Step', [model '/reference_step'], ...
        'Position', [50 80 90 110]);
    add_block('simulink/Continuous/State-Space', [model '/closed_loop_ss'], ...
        'Position', [160 65 300 125], ...
        'A', mat2str(Acl), ...
        'B', mat2str(bc), ...
        'C', mat2str(C), ...
        'D', mat2str(D));
    add_block('simulink/Sinks/Scope', [model '/state_scope'], ...
        'Position', [370 70 410 120]);

    add_line(model, 'reference_step/1', 'closed_loop_ss/1');
    add_line(model, 'closed_loop_ss/1', 'state_scope/1');
    set_param(model, 'StopTime', '5');
    save_system(model);
    disp(['Created Simulink model: ' model '.slx']);
end

function [A, b] = companion_pair_from_coefficients(aAscending)
    n = numel(aAscending);
    A = zeros(n);
    if n > 1
        A(1:n-1,2:n) = eye(n-1);
    end
    A(n,:) = -aAscending(:).';
    b = zeros(n,1);
    b(n) = 1;
end

function alphaAscending = desired_coefficients_from_poles(poles)
    desc = poly(poles);             % [1 alpha_{n-1} ... alpha0]
    alphaAscending = fliplr(desc(2:end));
end

function K = ccf_gain(aAscending, desiredPoles)
    alpha = desired_coefficients_from_poles(desiredPoles);
    if numel(alpha) ~= numel(aAscending)
        error('Number of desired poles must match system order.');
    end
    K = alpha - aAscending;
end
