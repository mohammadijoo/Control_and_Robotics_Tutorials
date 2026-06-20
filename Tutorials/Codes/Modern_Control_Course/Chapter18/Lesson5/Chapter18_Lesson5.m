% Chapter18_Lesson5.m
% Interpretation of Jordan form in control applications.
%
% Required for the main analysis:
%   MATLAB base functions: expm, rank
% Recommended:
%   Control System Toolbox: ctrb, obsv, ss, initial
%   Symbolic Math Toolbox: jordan, sym
% Optional:
%   Simulink for automatic construction of a State-Space block model

clear; clc; close all;

lambda = -1;
A = [lambda 1 0;
     0 lambda 1;
     0 0 lambda];

fprintf('A = Jordan block J_3(-1):\n');
disp(A);

t = 2;
E_closed = exp(lambda*t) * [1 t t^2/factorial(2);
                            0 1 t;
                            0 0 1];
E_numeric = expm(A*t);

fprintf('Closed-form exp(A t) at t=2:\n');
disp(E_closed);
fprintf('MATLAB expm(A t) at t=2:\n');
disp(E_numeric);
fprintf('Max absolute difference = %.3e\n', max(abs(E_closed - E_numeric), [], 'all'));

B_good = [0; 0; 1];
B_bad = [1; 0; 0];

Wc_good = ctrb(A, B_good);
Wc_bad = ctrb(A, B_bad);

fprintf('\nControllability rank with B_good = %d of %d\n', rank(Wc_good), size(A,1));
disp(Wc_good);
fprintf('Controllability rank with B_bad = %d of %d\n', rank(Wc_bad), size(A,1));
disp(Wc_bad);

C_good = [1 0 0];
C_bad = [0 0 1];

Wo_good = obsv(A, C_good);
Wo_bad = obsv(A, C_bad);

fprintf('\nObservability rank with C_good = %d of %d\n', rank(Wo_good), size(A,1));
disp(Wo_good);
fprintf('Observability rank with C_bad = %d of %d\n', rank(Wo_bad), size(A,1));
disp(Wo_bad);

% Free-response comparison for a generalized eigenvector initial condition.
x0 = [0; 0; 1];
time = linspace(0, 6, 121);
X = zeros(numel(time), 3);
for k = 1:numel(time)
    X(k,:) = (expm(A*time(k))*x0).';
end

figure('Name','Chapter18 Lesson5 Jordan-chain response');
plot(time, X, 'LineWidth', 1.5);
grid on;
xlabel('time');
ylabel('state components');
legend('x_1','x_2','x_3','Location','best');
title('Polynomial-times-exponential response of a Jordan chain');

% Symbolic Jordan decomposition when Symbolic Math Toolbox is available.
if license('test','Symbolic_Toolbox')
    fprintf('\nSymbolic Jordan form of A:\n');
    Js = jordan(sym(A));
    disp(Js);
end

% Minimal Simulink construction: continuous State-Space block with A,B,C,D.
if license('test','Simulink')
    mdl = 'Chapter18_Lesson5_Simulink';
    if bdIsLoaded(mdl)
        close_system(mdl, 0);
    end
    new_system(mdl);
    open_system(mdl);
    add_block('simulink/Continuous/State-Space', [mdl '/Jordan_Chain_State_Space']);
    set_param([mdl '/Jordan_Chain_State_Space'], ...
        'A', mat2str(A), ...
        'B', mat2str(B_good), ...
        'C', mat2str(C_good), ...
        'D', '0', ...
        'X0', mat2str(x0));
    add_block('simulink/Sinks/Scope', [mdl '/Scope']);
    add_line(mdl, 'Jordan_Chain_State_Space/1', 'Scope/1');
    save_system(mdl);
    fprintf('Created Simulink model: %s.slx\n', mdl);
end
