% Chapter9_Lesson2.m
% Eigenvalue-based stability criteria for LTI systems in MATLAB/Octave.
% Related MATLAB functions: eig, jordan (Symbolic Math Toolbox), expm, ss, isstable.

clear; clc;

A1 = [-1 0; 0 -2];
A2 = [0 -1; 1 0];
A3 = [0 1; 0 0];

Ad1 = [0.8 0; 0 0.5];
Ad2 = [1.02 0; 0 0.7];

reportContinuous("Continuous asymptotically stable example", A1);
reportContinuous("Continuous oscillatory borderline example", A2);
reportContinuous("Continuous defective zero-eigenvalue example", A3);
reportDiscrete("Discrete asymptotically stable example", Ad1);
reportDiscrete("Discrete unstable example", Ad2);

% If Control System Toolbox is available:
% sys = ss(A1, eye(2), eye(2), zeros(2));
% isstable(sys)

function reportContinuous(name, A)
    fprintf("\n============================================================\n");
    fprintf("%s\n", name);
    disp("A ="); disp(A);
    lambda = eig(A);
    disp("eigenvalues ="); disp(lambda);
    fprintf("classification: %s\n", classifyContinuous(A));

    sampleTimes = [0 1 2 5 10];
    fprintf("norm(expm(A*t),2) samples:\n");
    for t = sampleTimes
        fprintf("  t = %2g: %.8f\n", t, norm(expm(A*t), 2));
    end
end

function reportDiscrete(name, A)
    fprintf("\n============================================================\n");
    fprintf("%s\n", name);
    disp("A ="); disp(A);
    lambda = eig(A);
    disp("eigenvalues ="); disp(lambda);
    fprintf("classification: %s\n", classifyDiscrete(A));

    sampleSteps = [0 1 2 5 10];
    fprintf("norm(A^k,2) samples:\n");
    for k = sampleSteps
        fprintf("  k = %2g: %.8f\n", k, norm(A^k, 2));
    end
end

function label = classifyContinuous(A)
    tol = 1e-10;
    lambda = eig(A);
    maxReal = max(real(lambda));

    if maxReal < -tol
        label = "asymptotically stable";
    elseif maxReal > tol
        label = "unstable";
    else
        label = "borderline: check semisimplicity of imaginary-axis eigenvalues";
    end
end

function label = classifyDiscrete(A)
    tol = 1e-10;
    lambda = eig(A);
    rho = max(abs(lambda));

    if rho < 1 - tol
        label = "asymptotically stable";
    elseif rho > 1 + tol
        label = "unstable";
    else
        label = "borderline: check semisimplicity of unit-circle eigenvalues";
    end
end
