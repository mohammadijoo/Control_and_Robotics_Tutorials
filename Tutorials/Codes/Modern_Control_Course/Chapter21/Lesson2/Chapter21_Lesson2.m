% Chapter21_Lesson2.m
% Computation of transmission / invariant zeros from (A, B, C, D).
%
% Requires for tzero/ss:
%   Control System Toolbox
% Optional Simulink section:
%   Simulink

clear; clc;

% Minimal SISO example:
% G(s) = (s + 4)/(s^2 + 3s + 2), so the zero is -4.
A = [0 1; -2 -3];
B = [0; 1];
C = [4 1];
D = 0;

sys = ss(A, B, C, D);
zeros_builtin = tzero(sys);
disp('Transmission zeros from Control System Toolbox tzero:');
disp(zeros_builtin);

% Rosenbrock generalized eigenvalue pencil for square systems p == m:
[n, ~] = size(A);
[p, m] = size(D);
E = [eye(n), zeros(n, m); zeros(p, n), zeros(p, m)];
M = [A, B; -C, -D];
all_gen_eigs = eig(M, E);
finite_zeros = all_gen_eigs(isfinite(all_gen_eigs));

disp('Finite generalized eigenvalues of Rosenbrock pencil:');
disp(finite_zeros);

% If D is nonsingular, zeros are eigenvalues of A - B*inv(D)*C.
D2 = 1;
Az = A - B * (D2 \ C);
disp('Zeros for nonsingular D from eig(A - B*D^{-1}*C):');
disp(eig(Az));

% Optional: create a simple Simulink model containing a State-Space block.
% This block is useful for simulation, while tzero(sys) is the analysis command.
if license('test', 'Simulink')
    model = 'Chapter21_Lesson2_Simulink_Zero_Computation';
    if bdIsLoaded(model)
        close_system(model, 0);
    end
    new_system(model);
    add_block('simulink/Sources/Step', [model '/Step']);
    add_block('simulink/Continuous/State-Space', [model '/State-Space']);
    add_block('simulink/Sinks/Scope', [model '/Scope']);
    set_param([model '/State-Space'], ...
        'A', mat2str(A), 'B', mat2str(B), 'C', mat2str(C), 'D', mat2str(D));
    add_line(model, 'Step/1', 'State-Space/1');
    add_line(model, 'State-Space/1', 'Scope/1');
    save_system(model);
    disp(['Created Simulink model: ' model '.slx']);
end
