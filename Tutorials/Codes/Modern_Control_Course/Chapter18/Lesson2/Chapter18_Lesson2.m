% Chapter18_Lesson2.m
% Exact Jordan canonical form construction plus a state-space/Simulink check.
% Required for jordan: Symbolic Math Toolbox.
% Required for ss/sim: Control System Toolbox and Simulink.

clear; clc;

A = sym([ ...
     2  1  0  0  0  0; ...
     0  2  1 -1  1 -1; ...
     0  0  2  0  0  0; ...
     0  0  0  2 -3  4; ...
     0  0  0  0 -1  1; ...
     0  0  0  0  0 -1]);

disp('A ='); disp(A);

[V,J] = jordan(A);
disp('Jordan form J ='); disp(J);
disp('Verification inv(V)*A*V - J ='); disp(simplify(inv(V)*A*V - J));

lambdaValues = [sym(2), sym(-1)];
algebraicMultiplicities = [4, 2];

for idx = 1:numel(lambdaValues)
    lambda = lambdaValues(idx);
    alg = algebraicMultiplicities(idx);
    N = A - lambda*eye(size(A));

    nullities = zeros(1, size(A,1));
    for k = 1:size(A,1)
        nullities(k) = size(A,1) - rank(N^k);
    end

    fprintf('\nlambda = %s\n', char(lambda));
    fprintf('algebraic multiplicity = %d\n', alg);
    fprintf('nullities n_k = ');
    disp(nullities);

    % b_k = n_k - n_{k-1}: number of blocks of size at least k
    nWithZero = [0, nullities];
    bAtLeast = diff(nWithZero);
    bAtLeast = bAtLeast(1:find(nullities == alg, 1, 'first'));
    bAtLeast = [bAtLeast, 0];

    blockSizes = [];
    for k = 1:(numel(bAtLeast)-1)
        exactCount = bAtLeast(k) - bAtLeast(k+1);
        blockSizes = [blockSizes, k*ones(1, exactCount)]; %#ok<AGROW>
    end
    blockSizes = sort(blockSizes, 'descend');
    fprintf('Jordan block sizes = ');
    disp(blockSizes);
end

% State-space interpretation for x_dot = A x, y = x.
% Jordan form is used here as an analytical coordinate transformation;
% simulation should use A directly or numerically stable Schur-based methods.
try
    sys = ss(double(A), zeros(6,1), eye(6), zeros(6,1));
    disp('Continuous-time state-space model sys = ss(A,0,I,0):');
    disp(sys);
catch ME
    warning('Control System Toolbox state-space object was not created: %s', ME.message);
end

% Optional Simulink construction: a State-Space block with A as the state matrix.
try
    model = 'Chapter18_Lesson2_SimulinkModel';
    if bdIsLoaded(model)
        close_system(model, 0);
    end
    new_system(model);
    add_block('simulink/Continuous/State-Space', [model '/State-Space']);
    set_param([model '/State-Space'], ...
        'A', mat2str(double(A)), ...
        'B', mat2str(zeros(6,1)), ...
        'C', mat2str(eye(6)), ...
        'D', mat2str(zeros(6,1)));
    save_system(model);
    fprintf('Saved optional Simulink model: %s.slx\n', model);
catch ME
    warning('Simulink model was not created: %s', ME.message);
end
