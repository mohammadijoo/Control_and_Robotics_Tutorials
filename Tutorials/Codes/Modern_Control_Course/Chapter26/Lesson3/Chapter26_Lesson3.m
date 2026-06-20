% Chapter26_Lesson3.m
% Design of state-feedback gains for an augmented system with integral action.
% Requires Control System Toolbox for ctrb, place, ss, lsim.

clear; clc; close all;

% Plant: mass-spring-damper model
A = [0 1; -2 -0.6];
B = [0; 1];
C = [1 0];
D = 0;

n = size(A,1);
p = size(C,1);

% Integral state: zdot = r - y = r - Cx - Du
Aaug = [A zeros(n,p); -C zeros(p,p)];
Baug = [B; -D];
Eaug = [zeros(n,p); eye(p)];

Co = ctrb(Aaug, Baug);
fprintf('Augmented controllability rank: %d of %d\n', rank(Co), n+p);

poles = [-2 -2.5 -3];
Kaug = place(Aaug, Baug, poles);
Kx = Kaug(:,1:n);
Ki = Kaug(:,n+1:end);
Acl = Aaug - Baug*Kaug;

fprintf('Kaug = '); disp(Kaug);
fprintf('Kx = '); disp(Kx);
fprintf('Ki = '); disp(Ki);
fprintf('Closed-loop eigenvalues:\n'); disp(eig(Acl));

% Step-reference simulation: xadot = Acl xa + Eaug r
sys_cl = ss(Acl, Eaug, [C zeros(p,p); -Kaug], [zeros(p,p); zeros(size(B,2),p)]);
t = linspace(0,8,800);
r = ones(size(t));
resp = lsim(sys_cl, r, t);
y = resp(:,1);
u = resp(:,2);

figure;
plot(t, y, 'LineWidth', 1.5); hold on;
plot(t, r, '--', 'LineWidth', 1.0);
grid on; xlabel('time [s]'); ylabel('output');
legend('y','r'); title('State feedback with integral action');

figure;
plot(t, u, 'LineWidth', 1.5);
grid on; xlabel('time [s]'); ylabel('u');
title('Control effort');

% Optional Simulink skeleton construction.
% This programmatically creates a minimal model with a State-Space block for
% the augmented closed loop. Open the model and add scopes as needed.
model = 'Chapter26_Lesson3_Simulink';
if bdIsLoaded(model)
    close_system(model,0);
end
new_system(model);
open_system(model);
add_block('simulink/Sources/Step', [model '/reference_step'], 'Position', [80 80 120 110]);
add_block('simulink/Continuous/State-Space', [model '/augmented_closed_loop'], 'Position', [200 60 360 130]);
set_param([model '/augmented_closed_loop'], ...
    'A', mat2str(Acl), 'B', mat2str(Eaug), ...
    'C', mat2str([C zeros(p,p); -Kaug]), ...
    'D', mat2str(zeros(p+size(B,2),p)));
add_block('simulink/Sinks/Scope', [model '/scope_y_u'], 'Position', [430 70 470 120]);
add_line(model, 'reference_step/1', 'augmented_closed_loop/1');
add_line(model, 'augmented_closed_loop/1', 'scope_y_u/1');
save_system(model);
