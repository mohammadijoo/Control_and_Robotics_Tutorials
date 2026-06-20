% Chapter18_Lesson4.m
% Repeated eigenvalues and non-diagonalizable systems.
% This script compares MATLAB expm with the exact Jordan-block expression.

clear; clc; close all;

lambda = -0.4;
A = [lambda 1 0;
     0 lambda 1;
     0 0 lambda];

x0 = [1; -2; 1.5];

% Algebraic multiplicity is 3. Geometric multiplicity is dim null(A-lambda I).
geom_mult = size(A,1) - rank(A - lambda * eye(3));
fprintf('Geometric multiplicity of lambda = %.2f is %d\n', lambda, geom_mult);

t_grid = linspace(0, 8, 200);
X_expm = zeros(3, numel(t_grid));
X_jordan = zeros(3, numel(t_grid));

for k = 1:numel(t_grid)
    t = t_grid(k);

    Phi_expm = expm(A * t);
    Phi_jordan = exp(lambda * t) * [1, t, t^2/2;
                                    0, 1, t;
                                    0, 0, 1];

    X_expm(:, k) = Phi_expm * x0;
    X_jordan(:, k) = Phi_jordan * x0;
end

fprintf('Maximum difference between expm and Jordan formula: %.3e\n', ...
    max(abs(X_expm(:) - X_jordan(:))));

figure;
plot(t_grid, X_jordan(1,:), 'LineWidth', 1.5); hold on;
plot(t_grid, X_jordan(2,:), 'LineWidth', 1.5);
plot(t_grid, X_jordan(3,:), 'LineWidth', 1.5);
grid on;
xlabel('t');
ylabel('state components');
title('Response of a 3 x 3 Jordan block');
legend('x_1(t)', 'x_2(t)', 'x_3(t)', 'Location', 'best');

% Optional Simulink model construction, if Simulink is available.
% This builds a continuous State-Space block with the defective A matrix.
if exist('new_system', 'file') == 2
    mdl = 'Chapter18_Lesson4_Simulink';
    if ~bdIsLoaded(mdl)
        new_system(mdl);
    end
    open_system(mdl);
    add_block('simulink/Continuous/State-Space', [mdl '/Jordan State Space'], ...
        'A', mat2str(A), ...
        'B', mat2str(eye(3)), ...
        'C', mat2str(eye(3)), ...
        'D', mat2str(zeros(3)), ...
        'Position', [150 100 320 180]);
    save_system(mdl);
    fprintf('Optional Simulink model saved as %s.slx\n', mdl);
end
