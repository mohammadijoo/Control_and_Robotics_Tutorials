% Chapter26_Lesson4.m
% State-feedback with integral action for step and ramp reference inputs.
% Required MATLAB toolboxes: Control System Toolbox for place(), ss(), lsim().
% The augmented matrices are also shown explicitly for implementation clarity.

clear; clc; close all;

A = [0 1; -2 -3];
B = [0; 1];
C = [1 0];

%% Step reference: one error integrator, eta_dot = r - y
A_step = [A zeros(2,1); -C 0];
B_step = [B; 0];
E_step = [0; 0; 1];

fprintf('rank ctrb step augmented = %d / %d\n', rank(ctrb(A_step, B_step)), size(A_step,1));
K_step = place(A_step, B_step, [-4 -5 -6]);
disp('K_step = '); disp(K_step);

Acl_step = A_step - B_step*K_step;
Bcl_step = E_step;
Ccl_step = [C 0];
Dcl_step = 0;
sys_step = ss(Acl_step, Bcl_step, Ccl_step, Dcl_step);

%% Ramp reference: two error-integrator states
% eta1_dot = r - y, eta2_dot = eta1
A_ramp = [A zeros(2,2); -C 0 0; 0 0 1 0];
B_ramp = [B; 0; 0];
E_ramp = [0; 0; 1; 0];

fprintf('rank ctrb ramp augmented = %d / %d\n', rank(ctrb(A_ramp, B_ramp)), size(A_ramp,1));
K_ramp = place(A_ramp, B_ramp, [-3 -4 -5 -6]);
disp('K_ramp = '); disp(K_ramp);

Acl_ramp = A_ramp - B_ramp*K_ramp;
Bcl_ramp = E_ramp;
Ccl_ramp = [C 0 0];
Dcl_ramp = 0;
sys_ramp = ss(Acl_ramp, Bcl_ramp, Ccl_ramp, Dcl_ramp);

%% Simulation
T = linspace(0, 8, 801).';
r_step = ones(size(T));
r_ramp = T;
y_step = lsim(sys_step, r_step, T);
y_ramp = lsim(sys_ramp, r_ramp, T);

fprintf('Final step error = %.10g\n', r_step(end) - y_step(end));
fprintf('Final ramp error = %.10g\n', r_ramp(end) - y_ramp(end));

figure;
plot(T, r_step, '--', T, y_step, 'LineWidth', 1.5);
grid on; xlabel('time [s]'); ylabel('y(t)');
title('Step tracking with one error integrator');
legend('reference', 'output', 'Location', 'best');

figure;
plot(T, r_ramp, '--', T, y_ramp, 'LineWidth', 1.5);
grid on; xlabel('time [s]'); ylabel('y(t)');
title('Ramp tracking with two error integrators');
legend('reference', 'output', 'Location', 'best');

%% Simulink construction notes
% 1. Put the plant in a State-Space block with matrices A, B, C, D = 0.
% 2. For step tracking, feed e = r - y into one Integrator block eta.
% 3. Implement u = -K_step*[x1; x2; eta] using a Gain block.
% 4. For ramp tracking, cascade two Integrator blocks driven by e:
%       eta1_dot = e, eta2_dot = eta1.
% 5. Implement u = -K_ramp*[x1; x2; eta1; eta2].
