% Chapter19_Lesson2.m
% Observable/unobservable subspaces for a continuous-time LTI system.
% Toolboxes/libraries covered:
%   - Control System Toolbox: obsv, ss, initial.
%   - Base MATLAB: matrix powers, rank, null, orth.
%   - Simulink: optional State-Space block model construction.

clear; clc;

A = [ 0  1  0  0;
     -2 -3  0  0;
      0  0  0  1;
      0  0 -5 -1];
B = [0; 1; 0; 1];
C = [1 0 0 0];
D = 0;

n = size(A,1);

% From-scratch observability matrix
O = [];
Ak = eye(n);
for k = 0:n-1
    O = [O; C*Ak]; %#ok<AGROW>
    Ak = Ak*A;
end

fprintf('Observability matrix O_n =\n'); disp(O);
fprintf('rank(O_n) = %d out of n = %d\n\n', rank(O), n);

% Control System Toolbox equivalent, when available:
if exist('obsv','file') == 2
    fprintf('Control System Toolbox obsv(A,C) matches from-scratch O: %d\n\n', norm(obsv(A,C)-O,'fro') < 1e-12);
end

Qu = null(O, 'r');        % rational-looking basis for ker(O)
Qo = orth(O');            % basis for row space of O
Q  = [Qo Qu];             % x = Q z, for this example Q is nonsingular/orthonormal enough

fprintf('Basis Qu for unobservable subspace ker(O_n):\n'); disp(Qu);
fprintf('A-invariance residual ||(I-Qu*pinv(Qu))*A*Qu||_F = %.3e\n\n', norm((eye(n)-Qu*pinv(Qu))*A*Qu, 'fro'));

Abar = Q\A*Q;             % equivalent to inv(Q)*A*Q, better numerically
Bbar = Q\B;
Cbar = C*Q;

fprintf('Abar = inv(Q)*A*Q =\n'); disp(Abar);
fprintf('Bbar = inv(Q)*B =\n'); disp(Bbar);
fprintf('Cbar = C*Q =\n'); disp(Cbar);

% Demonstrate output indistinguishability using initial response.
if exist('ss','file') == 2
    sys = ss(A,B,C,D);
    t = linspace(0,5,200);
    x_observable = [1;0;0;0];
    x_hidden = [0;0;1;-1];
    y1 = initial(sys, x_observable, t);
    y2 = initial(sys, x_observable + x_hidden, t);
    fprintf('max |y1(t)-y2(t)| over t-grid = %.3e\n\n', max(abs(y1-y2)));
end

% Optional Simulink model construction.  Requires Simulink license.
if exist('simulink','file') == 2
    model = 'Chapter19_Lesson2_Simulink';
    if bdIsLoaded(model), close_system(model, 0); end
    new_system(model);
    add_block('simulink/Continuous/State-Space', [model '/State-Space Pair']);
    set_param([model '/State-Space Pair'], 'A', 'A', 'B', 'B', 'C', 'C', 'D', 'D');
    add_block('simulink/Sources/Step', [model '/Input Step']);
    add_block('simulink/Sinks/Scope', [model '/Output Scope']);
    add_line(model, 'Input Step/1', 'State-Space Pair/1');
    add_line(model, 'State-Space Pair/1', 'Output Scope/1');
    save_system(model);
    fprintf('Created optional Simulink model: %s.slx\n', model);
end
