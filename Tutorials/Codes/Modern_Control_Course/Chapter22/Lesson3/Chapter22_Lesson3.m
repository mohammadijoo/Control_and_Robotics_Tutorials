% Chapter22_Lesson3.m
% State Feedback vs Output Feedback (Concept Only)
%
% This script compares full-state feedback u = -K*x + r with static output
% feedback u = -F*y + r for a small continuous-time LTI plant.

clear; clc;

A = [0 1; -2 -0.4];
B = [0; 1];
C_full = eye(2);
C_position = [1 0];

K = [4 2.6];
A_state = A - B*K;

F_position = 4;
A_output_position = A - B*F_position*C_position;

F_full = K;
A_output_full = A - B*F_full*C_full;

print_summary(A, 'A_open_loop');
print_summary(A_state, 'A_state_feedback');
print_summary(A_output_position, 'A_static_output_feedback_position_only');
print_summary(A_output_full, 'A_static_output_feedback_full_output');

rank_C = rank(C_position);
rank_augmented = rank([C_position; K]);
fprintf('C_position rank: %d\n', rank_C);
fprintf('rank([C_position; K]): %d\n', rank_augmented);
fprintf('K implementable as F*C_position? %d\n', rank_augmented == rank_C);

F_ls = K*C_position'/(C_position*C_position');
K_projected = F_ls*C_position;
disp('Least-squares F for output feedback:'); disp(F_ls);
disp('Projected effective gain F*C:'); disp(K_projected);

function print_summary(M, name)
    fprintf('%s =\n', name);
    disp(M);
    lambda = eig(M);
    fprintf('eigenvalues(%s) =\n', name);
    disp(lambda);
    fprintf('stable? %d\n\n', all(real(lambda) < 0));
end
