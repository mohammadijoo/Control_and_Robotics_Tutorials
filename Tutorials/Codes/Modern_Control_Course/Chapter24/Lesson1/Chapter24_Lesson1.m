% Chapter24_Lesson1.m
% Existence conditions for MIMO pole assignment in MATLAB/Simulink.
% Requires Control System Toolbox for ctrb and place.

clear; clc;

A = [ 0  1  0;
      0  0  1;
     -1 -5 -6 ];

B = [0 0;
     1 0;
     0 1];

n = size(A, 1);
C = ctrb(A, B);
fprintf('rank(ctrb(A,B)) = %d\n', rank(C));
fprintf('Kalman controllable = %d\n', rank(C) == n);

% PBH test at eigenvalues of A.
lambdaA = eig(A);
pbh_ok = true;
for k = 1:length(lambdaA)
    M = [lambdaA(k) * eye(n) - A, B];
    r = rank(M);
    fprintf('PBH rank at lambda = %.6g%+.6gi is %d\n', real(lambdaA(k)), imag(lambdaA(k)), r);
    pbh_ok = pbh_ok && (r == n);
end
fprintf('PBH controllable = %d\n', pbh_ok);

% MIMO pole assignment.
desired_poles = [-2 -3 -4];
K = place(A, B, desired_poles);
fprintf('K from place(A,B,desired_poles):\n');
disp(K);

Acl = A - B * K;
fprintf('Closed-loop eigenvalues:\n');
disp(eig(Acl));

% A non-controllable example with an unstable uncontrollable mode.
A_bad = diag([1 -2 -3]);
B_bad = [0; 1; 1];
C_bad = ctrb(A_bad, B_bad);
fprintf('\nBad pair rank = %d\n', rank(C_bad));
M_bad = [1 * eye(3) - A_bad, B_bad];
fprintf('PBH rank at unstable lambda=1 is %d, so the pair is not stabilizable.\n', rank(M_bad));

% Simulate the closed-loop initial response.
sys_cl = ss(Acl, zeros(n, 1), eye(n), zeros(n, 1));
t = linspace(0, 6, 400);
x0 = [1; -0.5; 0.75];
[y, t, x] = initial(sys_cl, x0, t);
figure;
plot(t, x, 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('States');
title('Closed-loop state response after MIMO pole assignment');
legend('x_1', 'x_2', 'x_3');

% Optional Simulink construction: creates a State-Space block for A-BK.
if exist('simulink', 'file') == 4
    model = 'Chapter24_Lesson1_Simulink';
    if bdIsLoaded(model)
        close_system(model, 0);
    end
    new_system(model);
    open_system(model);
    add_block('simulink/Sources/In1', [model '/zero_input'], 'Position', [40 80 70 100]);
    add_block('simulink/Continuous/State-Space', [model '/closed_loop_ss'], 'Position', [140 55 300 125]);
    add_block('simulink/Sinks/Out1', [model '/states'], 'Position', [360 80 390 100]);
    set_param([model '/closed_loop_ss'], 'A', 'Acl', 'B', 'zeros(3,1)', 'C', 'eye(3)', 'D', 'zeros(3,1)', 'X0', '[1; -0.5; 0.75]');
    add_line(model, 'zero_input/1', 'closed_loop_ss/1');
    add_line(model, 'closed_loop_ss/1', 'states/1');
    set_param(model, 'StopTime', '6');
    save_system(model);
    fprintf('Created Simulink model: %s.slx\n', model);
end
