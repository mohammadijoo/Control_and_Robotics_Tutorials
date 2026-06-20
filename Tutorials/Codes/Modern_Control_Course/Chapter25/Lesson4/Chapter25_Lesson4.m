% Chapter25_Lesson4.m
% Impact of Model Uncertainty on State-Feedback Designs
% Requires Control System Toolbox for place(). Simulink section is optional.

clear; clc; close all;
rng(25);

A = [0 1; -2 -0.5];
B = [0; 1];
C = [1 0];
D = 0;

poles = [-2 -3];
K = place(A, B, poles);
Acl = A - B*K;

disp('Nominal K ='); disp(K);
disp('Nominal closed-loop eigenvalues ='); disp(eig(Acl));

% Lyapunov sufficient perturbation bound
Q = eye(2);
P = lyap(Acl', Q);  % solves Acl'*P + P*Acl = -Q
bound = min(eig(Q))/(2*norm(P,2));
fprintf('Sufficient perturbation bound ||DeltaAcl||_2 < %.6f\n', bound);

N = 5000;
sigmaA = 0.05;
rhoMax = 0.25;
maxReal = zeros(N,1);

for k = 1:N
    dA = sigmaA*randn(2,2);
    rho = -rhoMax + 2*rhoMax*rand;
    Atrue = A + dA;
    Btrue = B*(1 + rho);
    Acl_true = Atrue - Btrue*K;
    maxReal(k) = max(real(eig(Acl_true)));
end

fprintf('Unstable samples: %d out of %d\n', sum(maxReal >= 0), N);
fprintf('Worst observed max real part: %.6f\n', max(maxReal));
fprintf('95th percentile max real part: %.6f\n', prctile(maxReal,95));

figure;
histogram(maxReal, 50);
xline(0, '--');
xlabel('max real part of closed-loop eigenvalues');
ylabel('sample count');
title('Closed-loop pole movement under model uncertainty');
grid on;

% Optional Simulink construction: nominal and perturbed state-space blocks.
% This creates a minimal model if Simulink is installed.
try
    mdl = 'Chapter25_Lesson4_Simulink';
    if bdIsLoaded(mdl), close_system(mdl, 0); end
    new_system(mdl); open_system(mdl);
    add_block('simulink/Sources/Step', [mdl '/Step'], 'Position', [50 80 80 110]);
    add_block('simulink/Continuous/State-Space', [mdl '/Nominal Closed Loop'], ...
        'A', 'Acl', 'B', 'B', 'C', 'C', 'D', 'D', 'Position', [160 55 320 135]);
    add_block('simulink/Sinks/Scope', [mdl '/Scope'], 'Position', [390 75 420 115]);
    add_line(mdl, 'Step/1', 'Nominal Closed Loop/1');
    add_line(mdl, 'Nominal Closed Loop/1', 'Scope/1');
    save_system(mdl);
    disp(['Created optional Simulink model: ' mdl '.slx']);
catch ME
    disp('Simulink model was not created. Reason:');
    disp(ME.message);
end
