% Chapter19_Lesson3.m
% Kalman Decomposition: block structure of A, B, C
%
% Block order:
%   [ controllable-observable,
%     controllable-unobservable,
%     uncontrollable-observable,
%     uncontrollable-unobservable ]

clear; clc;

A = [-1.0  0.0  0.7  0.0;
      0.2 -2.0  0.3 -0.4;
      0.0  0.0 -3.0  0.0;
      0.0  0.0  0.6 -4.0];

B = [1.0; 0.5; 0.0; 0.0];
C = [2.0 0.0 -1.0 0.0];

[T,Abar,Bbar,Cbar,dims] = kalman_decomposition_lesson(A,B,C);

disp('Block dimensions [co, c_unobs, unctrl_obs, unctrl_unobs]:');
disp(dims);
disp('Abar ='); disp(Abar);
disp('Bbar ='); disp(Bbar);
disp('Cbar ='); disp(Cbar);

% Optional verification of the input-output transfer function:
% If Control System Toolbox is available:
% sys_full = ss(A,B,C,0);
% sys_minimal = minreal(sys_full);
% tf(sys_minimal)

function [T,Abar,Bbar,Cbar,dims] = kalman_decomposition_lesson(A,B,C)
    n = size(A,1);
    R = orth(controllability_matrix_lesson(A,B));
    N = orth(null(observability_matrix_lesson(A,C)));

    V2 = intersection_basis_lesson(R,N);  % controllable and unobservable
    V1 = complement_inside_lesson(R,V2);  % controllable-observable quotient
    V4 = complement_inside_lesson(N,V2);  % uncontrollable-unobservable quotient

    V124 = [V1 V2 V4];
    completed = append_independent_lesson(V124, eye(n), n);
    V3 = completed(:, size(V124,2)+1:end); % uncontrollable-observable quotient

    T = [V1 V2 V3 V4];

    Abar = T \ (A*T);
    Bbar = T \ B;
    Cbar = C*T;

    dims = [size(V1,2), size(V2,2), size(V3,2), size(V4,2)];
end

function R = controllability_matrix_lesson(A,B)
    n = size(A,1);
    R = [];
    Ak = eye(n);
    for k = 1:n
        R = [R Ak*B]; %#ok<AGROW>
        Ak = A*Ak;
    end
end

function O = observability_matrix_lesson(A,C)
    n = size(A,1);
    O = [];
    Ak = eye(n);
    for k = 1:n
        O = [O; C*Ak]; %#ok<AGROW>
        Ak = A*Ak;
    end
end

function W = intersection_basis_lesson(U,V)
    if isempty(U) || isempty(V)
        W = zeros(size(U,1),0);
        return;
    end
    K = null([U -V]);
    if isempty(K)
        W = zeros(size(U,1),0);
        return;
    end
    alpha = K(1:size(U,2),:);
    W = orth(U*alpha);
end

function extra = complement_inside_lesson(container, sub)
    completed = append_independent_lesson(sub, container, size(container,2));
    extra = completed(:, size(sub,2)+1:end);
end

function M = append_independent_lesson(current, candidates, targetDim)
    M = current;
    r = rank(M);
    for j = 1:size(candidates,2)
        trial = [M candidates(:,j)]; %#ok<AGROW>
        if rank(trial) > r
            M = trial;
            r = r + 1;
            if r >= targetDim
                break;
            end
        end
    end
end
