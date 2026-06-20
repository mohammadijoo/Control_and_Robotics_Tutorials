% Chapter11_Lesson2.m
% PBH controllability test for continuous-time LTI systems.
% Run: Chapter11_Lesson2

function Chapter11_Lesson2()
    A = diag([0, -1, -2]);
    B_bad = [0; 1; 1];
    B_good = [1; 1; 1];

    fprintf('\nExample 1: unactuated first mode\n');
    report_pbh(A, B_bad);

    fprintf('\nExample 2: all modes actuated\n');
    report_pbh(A, B_good);

    A_rep = diag([1, 1, 2]);
    B_rep_bad = [1; 1; 1];
    B_rep_good = [1 0; 0 1; 1 1];

    fprintf('\nExample 3: repeated eigenvalue, one input is insufficient\n');
    report_pbh(A_rep, B_rep_bad);

    fprintf('\nExample 4: repeated eigenvalue, two inputs cover the eigenspace\n');
    report_pbh(A_rep, B_rep_good);
end

function report_pbh(A, B)
    n = size(A, 1);
    C = local_ctrb(A, B);
    fprintf('Kalman controllability rank = %d/%d\n', rank(C), n);

    % If Control System Toolbox is available, this should match local_ctrb.
    if exist('ctrb', 'file') == 2
        fprintf('Control System Toolbox ctrb rank = %d/%d\n', rank(ctrb(A, B)), n);
    end

    [ok, table_out] = pbh_rank_test(A, B, 1e-9);
    disp(table_out);
    fprintf('PBH controllable? %d\n', ok);
end

function [ok, table_out] = pbh_rank_test(A, B, tol)
    n = size(A, 1);
    eigenvalues = eig(A);
    lambdas = unique(round(eigenvalues, 10), 'stable');
    ranks = zeros(length(lambdas), 1);
    passed = false(length(lambdas), 1);

    ok = true;
    for k = 1:length(lambdas)
        lambda = lambdas(k);
        M = [lambda * eye(n) - A, B];
        ranks(k) = rank(M, tol);
        passed(k) = (ranks(k) == n);
        ok = ok && passed(k);
    end

    table_out = table(lambdas, ranks, passed, ...
        'VariableNames', {'lambda', 'PBH_rank', 'passed'});
end

function C = local_ctrb(A, B)
    n = size(A, 1);
    C = B;
    Ak = eye(n);
    for k = 1:n-1
        Ak = Ak * A;
        C = [C, Ak * B]; %#ok<AGROW>
    end
end
