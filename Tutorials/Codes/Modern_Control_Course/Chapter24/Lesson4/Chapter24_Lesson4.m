% Chapter24_Lesson4.m
% Partial Pole Placement and Restricted Eigenstructure
% Requires: base MATLAB. Control System Toolbox is used only for ss/eig display.

clear; clc;

A = diag([0.2, 0.6, -0.5, -1.0]);
B = [1.0 0.0;
     0.3 1.0;
     0.2 0.4;
     0.1 0.2];

assigned_lambdas = [-2.0, -3.0];
G = [1.0 0.0;
     0.0 1.0];

V = zeros(4,2);
for i = 1:2
    lam = assigned_lambdas(i);
    g = G(:,i);
    V(:,i) = (A - lam*eye(4)) \ (B*g);
end

Nkeep = [0 0;
         0 0;
         1 0;
         0 1];

X = [V Nkeep];
Y = [G zeros(2,2)];
K = Y / X;                    % K X = Y
Acl = A - B*K;

fprintf('K =\n'); disp(K);
fprintf('Closed-loop eigenvalues =\n'); disp(eig(Acl).');
fprintf('cond(X) = %.6f\n', cond(X));
fprintf('Assigned-mode residual = %.6e\n', norm(Acl*V - V*diag(assigned_lambdas)));
fprintf('Preservation residual ||K*Nkeep|| = %.6e\n', norm(K*Nkeep));

% Restricted eigenstructure: require x3 = x4 for lambda = -2.
H = [0 0 1 -1];
lam = -2.0;
M = (A - lam*eye(4)) \ B;
S = H*M;
g_restricted = null(S);
g_restricted = g_restricted(:,1);
v_restricted = M*g_restricted;

fprintf('\nS = H*(A-lambda*I)^(-1)*B =\n'); disp(S);
fprintf('Restricted g =\n'); disp(g_restricted);
fprintf('Restricted v =\n'); disp(v_restricted);
fprintf('H*v =\n'); disp(H*v_restricted);

% Optional Simulink model creation for the closed-loop matrix.
% This creates a compact state-space simulation model if Simulink is installed.
if exist('new_system', 'file') == 2
    model = 'Chapter24_Lesson4_simulink';
    if bdIsLoaded(model)
        close_system(model, 0);
    end
    new_system(model);
    open_system(model);
    add_block('simulink/Sources/Step', [model '/StepInput'], 'Position', [60 90 90 120]);
    add_block('simulink/Continuous/State-Space', [model '/ClosedLoopSS'], 'Position', [160 70 310 150]);
    add_block('simulink/Sinks/Scope', [model '/Scope'], 'Position', [380 85 430 135]);
    set_param([model '/ClosedLoopSS'], ...
        'A', 'Acl', 'B', 'B(:,1)', 'C', 'eye(4)', 'D', 'zeros(4,1)');
    add_line(model, 'StepInput/1', 'ClosedLoopSS/1');
    add_line(model, 'ClosedLoopSS/1', 'Scope/1');
    set_param(model, 'StopTime', '8');
    save_system(model);
    fprintf('Created optional Simulink model: %s.slx\n', model);
end
