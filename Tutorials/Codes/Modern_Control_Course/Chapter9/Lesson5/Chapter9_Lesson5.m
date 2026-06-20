% Chapter9_Lesson5.m
%
% Modern Control — Chapter 9, Lesson 5
% Examples of Stable, Marginal, and Unstable State Matrices
%
% This MATLAB script classifies representative continuous-time LTI matrices
% for x_dot = A x, computes eigenvalues, and plots selected state norms.

clear; clc; close all;

examples = {
    'Stable diagonal',              [-2 0; 0 -5];
    'Stable damped oscillator',     [0 1; -4 -2];
    'Stable but nonnormal',         [-1 50; 0 -2];
    'Marginal oscillator',          [0 1; -1 0];
    'Marginal with semisimple zero',[0 0; 0 -2];
    'Unstable positive eigenvalue', [1 0; 0 -2];
    'Unstable defective zero',      [0 1; 0 0]
};

for k = 1:size(examples,1)
    name = examples{k,1};
    A = examples{k,2};
    fprintf('\n%s\n', name);
    fprintf('%s\n', repmat('-',1,length(name)));
    disp('A ='); disp(A);
    disp('eigenvalues ='); disp(eig(A).');
    fprintf('classification = %s\n', classifyContinuousLTI(A));
end

% Exact simulation using x(t) = expm(A t) x0.
t = linspace(0,10,500);
x0 = [1; 1];

selected = {
    'Stable damped oscillator', [0 1; -4 -2];
    'Marginal oscillator',      [0 1; -1 0];
    'Unstable defective zero',  [0 1; 0 0]
};

figure;
hold on;
for k = 1:size(selected,1)
    label = selected{k,1};
    A = selected{k,2};
    normx = zeros(size(t));
    for i = 1:numel(t)
        x = expm(A*t(i))*x0;
        normx(i) = norm(x,2);
    end
    plot(t,normx,'DisplayName',label,'LineWidth',1.5);
end
grid on;
xlabel('time t');
ylabel('state norm ||x(t)||_2');
title('Stable, Marginal, and Unstable State-Matrix Examples');
legend('Location','best');

function cls = classifyContinuousLTI(A)
    tol = 1e-9;
    lambda = eig(A);
    alpha = max(real(lambda));

    if alpha < -tol
        cls = 'asymptotically stable';
        return;
    elseif alpha > tol
        cls = 'unstable';
        return;
    end

    % Boundary case: check semisimplicity of eigenvalues on imaginary axis.
    n = size(A,1);
    used = false(numel(lambda),1);

    for i = 1:numel(lambda)
        if used(i)
            continue;
        end

        close = abs(lambda - lambda(i)) < 1e-7;
        used(close) = true;
        algebraicMultiplicity = sum(close);

        if abs(real(lambda(i))) <= 1e-7
            geometricMultiplicity = n - rank(A - lambda(i)*eye(n), 1e-7);
            if geometricMultiplicity < algebraicMultiplicity
                cls = 'unstable';
                return;
            end
        end
    end

    cls = 'marginally stable';
end
