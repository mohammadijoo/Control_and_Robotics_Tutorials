% Chapter30_Lesson4.m
% Integrated case study: mass-spring-damper model to state-feedback controller
% Libraries: base MATLAB for matrix work; Control System Toolbox for ss/place/lsim;
% Simulink section is optional and creates a closed-loop State-Space block.

clear; clc; close all;

m = 1.2;      % kg
b = 0.8;      % N*s/m
k = 3.0;      % N/m
zeta = 0.70;
Ts = 2.0;
omega_n = 4/(zeta*Ts);

A = [0 1; -k/m -b/m];
B = [0; 1/m];
C = [1 0];
D = 0;

Wc = [B A*B];
fprintf('rank(Wc) = %d\n', rank(Wc));

% Manual pole-matching formula for this second-order plant.
k1 = m*omega_n^2 - k;
k2 = 2*zeta*omega_n*m - b;
K_manual = [k1 k2];

% Optional toolbox verification using place.
desiredPoles = roots([1 2*zeta*omega_n omega_n^2]);
if exist('place', 'file') == 2
    K_place = place(A, B, desiredPoles);
else
    K_place = K_manual;
end

K = K_manual;
Acl = A - B*K;
Nbar = -1/(C*(Acl\B));
Bcl = B*Nbar;

fprintf('K_manual = [%g %g]\n', K_manual(1), K_manual(2));
fprintf('K_place  = [%g %g]\n', K_place(1), K_place(2));
fprintf('Nbar = %g\n', Nbar);
disp('closed-loop poles ='); disp(eig(Acl));

% Simulate the closed-loop reference response.
t = 0:0.002:6;
r = ones(size(t));
if exist('ss', 'file') == 2 && exist('lsim', 'file') == 2
    sys_cl = ss(Acl, Bcl, C, D);
    y = lsim(sys_cl, r, t);
else
    f = @(tt, x) Acl*x + Bcl;
    [t_ode, x_ode] = ode45(f, t, [0; 0]);
    y = x_ode*C.';
    t = t_ode;
end

figure;
plot(t, y, 'LineWidth', 1.5); grid on;
xlabel('time (s)'); ylabel('position output');
title('Chapter 30 Lesson 4: closed-loop step response');

% Optional Simulink model for closed-loop state-space response.
% It uses the already-designed closed-loop matrices Acl and Bcl.
try
    if exist('new_system', 'file') == 4
        model = 'Chapter30_Lesson4_Simulink';
        if bdIsLoaded(model)
            close_system(model, 0);
        end
        new_system(model);
        add_block('simulink/Sources/Step', [model '/reference_step'], 'Position', [50 80 100 110]);
        add_block('simulink/Continuous/State-Space', [model '/closed_loop_state_space'], ...
            'A', mat2str(Acl), 'B', mat2str(Bcl), 'C', mat2str(C), 'D', mat2str(D), ...
            'Position', [170 68 310 122]);
        add_block('simulink/Sinks/Scope', [model '/scope'], 'Position', [380 75 430 115]);
        add_line(model, 'reference_step/1', 'closed_loop_state_space/1');
        add_line(model, 'closed_loop_state_space/1', 'scope/1');
        set_param(model, 'StopTime', '6');
        save_system(model);
        fprintf('Simulink model saved: %s.slx\n', model);
    end
catch ME
    fprintf('Simulink model creation skipped: %s\n', ME.message);
end
