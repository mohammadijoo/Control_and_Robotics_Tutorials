% Chapter14_Lesson3.m
% Duality Between Controllability and Observability
% MATLAB / GNU Octave compatible scratch implementation.
% If you have Control System Toolbox, compare with ctrb(A,B) and obsv(A,C).

clear; clc;

A = [0 1 0;
     0 0 1;
    -2 -3 -4];

B = [0; 0; 1];
C = [1 0 0];

Ctrb = local_ctrb(A, B);
Obsv = local_obsv(A, C);

Ctrb_dual = local_ctrb(A.', C.');
Obsv_dual = local_obsv(A.', B.');

fprintf('rank Ctrb(A,B)       = %d of %d\n', rank(Ctrb), size(A,1));
fprintf('rank Obsv(A,C)       = %d of %d\n', rank(Obsv), size(A,1));
fprintf('rank Ctrb(A'',C'')    = %d of %d\n', rank(Ctrb_dual), size(A,1));
fprintf('rank Obsv(A'',B'')    = %d of %d\n', rank(Obsv_dual), size(A,1));

fprintf('\nDuality identities:\n');
fprintf('norm(Obsv - Ctrb_dual'') = %.3e\n', norm(Obsv - Ctrb_dual.', 'fro'));
fprintf('norm(Ctrb'' - Obsv_dual) = %.3e\n', norm(Ctrb.' - Obsv_dual, 'fro'));

% Simulink note:
% Build a State-Space block with matrices A, B, C, D=0.
% To simulate the dual system, use A_dual=A.', B_dual=C.', C_dual=B.', D_dual=0.
% Compare state/output responses and rank tests in MATLAB before constructing
% an observer or pole-placement design.

function Ctrb = local_ctrb(A, B)
    n = size(A, 1);
    Ctrb = [];
    Ak = eye(n);
    for k = 0:n-1
        Ctrb = [Ctrb, Ak * B]; %#ok<AGROW>
        Ak = Ak * A;
    end
end

function Obsv = local_obsv(A, C)
    n = size(A, 1);
    Obsv = [];
    Ak = eye(n);
    for k = 0:n-1
        Obsv = [Obsv; C * Ak]; %#ok<AGROW>
        Ak = Ak * A;
    end
end
