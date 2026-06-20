% Chapter12_Lesson1.m
% Definition and computation of the controllability Gramian.
%
% This script demonstrates:
%   1. finite-horizon Gramian by numerical integration,
%   2. the differential equation dW/dt = A W + W A' + B B',
%   3. infinite-horizon Gramian by the Lyapunov equation,
%   4. optional Control System Toolbox commands,
%   5. optional Simulink model creation for the state equation.

clear; clc;

A = [0 1; -2 -3];
B = [0; 1];
T = 2.0;

fprintf('Kalman controllability matrix:\n');
Cmat = ctrb(A, B);
disp(Cmat);
fprintf('rank(C) = %d\n', rank(Cmat));

% Finite-horizon definition:
% Wc(T) = integral_0^T expm(A*tau)*B*B'*expm(A'*tau) dtau
integrand = @(tau) reshape(expm(A*tau) * B * B' * expm(A'*tau), [], 1);
wvec = integral(integrand, 0, T, 'ArrayValued', true, 'AbsTol', 1e-11, 'RelTol', 1e-11);
WcT = reshape(wvec, size(A));
WcT = 0.5 * (WcT + WcT');

fprintf('\nFinite-horizon Gramian Wc(T):\n');
disp(WcT);
fprintf('eig(Wc(T)):\n');
disp(eig(WcT));
fprintf('rank(Wc(T)) = %d\n', rank(WcT));

% Alternative: integrate the Gramian differential equation.
odefun = @(t, w) reshape(A*reshape(w, 2, 2) + reshape(w, 2, 2)*A' + B*B', [], 1);
[~, Wtraj] = ode45(odefun, [0 T], zeros(4,1));
Wode = reshape(Wtraj(end, :).', 2, 2);
Wode = 0.5 * (Wode + Wode');

fprintf('\nFinite-horizon Gramian from ODE integration:\n');
disp(Wode);

% Infinite-horizon Gramian for Hurwitz A:
% A*W + W*A' + B*B' = 0
Winf = lyap(A, B*B');
fprintf('\nInfinite-horizon Gramian Wc(infinity):\n');
disp(Winf);

% Optional Control System Toolbox command:
sys = ss(A, B, eye(2), zeros(2,1));
try
    Wtoolbox = gram(sys, 'c');
    fprintf('\nControl System Toolbox gram(sys, ''c'') result:\n');
    disp(Wtoolbox);
catch ME
    fprintf('\ngram(sys, ''c'') was not available: %s\n', ME.message);
end

% Optional Simulink construction:
% The Gramian is not a standard Simulink block; however, one may simulate
% xdot = A*x + B*u and estimate reachability effects from impulse-like inputs.
% This block creates a minimal state-space model if Simulink is installed.
try
    model = 'Chapter12_Lesson1_Simulink_StateSpace';
    if bdIsLoaded(model)
        close_system(model, 0);
    end
    new_system(model);
    add_block('simulink/Sources/Step', [model '/Step']);
    add_block('simulink/Continuous/State-Space', [model '/State-Space']);
    add_block('simulink/Sinks/Scope', [model '/Scope']);

    set_param([model '/State-Space'], ...
        'A', mat2str(A), ...
        'B', mat2str(B), ...
        'C', mat2str(eye(2)), ...
        'D', mat2str(zeros(2,1)));

    add_line(model, 'Step/1', 'State-Space/1');
    add_line(model, 'State-Space/1', 'Scope/1');
    save_system(model);
    fprintf('\nCreated optional Simulink model: %s.slx\n', model);
catch ME
    fprintf('\nSimulink model creation skipped: %s\n', ME.message);
end
