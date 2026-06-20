% Chapter30_Lesson2.m
% Coordinate selection, diagonal scaling, conditioning, and Control System
% Toolbox workflows for state-space models.

clear; clc;

A = [0        1        0;
    -2.0e3   -5.0e1   8.0e4;
     0       -2.0e-2 -4.0e3];

B = [0; 0; 2.0e3];
C = [1 0 0];
D = 0;

% Engineering nominal magnitudes for x = [position; velocity; current].
x_nom = [1.0e-3; 1.0e-1; 1.0e1];

% z = inv(S) x, with S = diag(x_nom).
S = diag(x_nom);
Sinv = inv(S);

Az = Sinv*A*S;
Bz = Sinv*B;
Cz = C*S;

fprintf('Eigenvalues of A:\n');
disp(eig(A));
fprintf('Eigenvalues of Az:\n');
disp(eig(Az));

Mc  = ctrb(A, B);
Mcz = ctrb(Az, Bz);
Mo  = obsv(A, C);
Moz = obsv(Az, Cz);

fprintf('cond(A)             = %.6e\n', cond(A));
fprintf('cond(Az)            = %.6e\n', cond(Az));
fprintf('cond(ctrb(A,B))     = %.6e\n', cond(Mc));
fprintf('cond(ctrb(Az,Bz))   = %.6e\n', cond(Mcz));
fprintf('cond(obsv(A,C))     = %.6e\n', cond(Mo));
fprintf('cond(obsv(Az,Cz))   = %.6e\n', cond(Moz));

% Stable Gramians.
Wc  = gram(A, B, 'c');
Wo  = gram(A, C, 'o');
Wcz = gram(Az, Bz, 'c');
Woz = gram(Az, Cz, 'o');

fprintf('cond(Wc),  cond(Wo)  = %.6e, %.6e\n', cond(Wc),  cond(Wo));
fprintf('cond(Wcz), cond(Woz) = %.6e, %.6e\n', cond(Wcz), cond(Woz));

% Pole placement in scaled coordinates, then transform the gain back.
p = [-20 -35 -1200];
Kz = place(Az, Bz, p);
Kx = Kz*Sinv;

fprintf('Kz =\n');
disp(Kz);
fprintf('Kx = Kz*inv(S) =\n');
disp(Kx);
fprintf('eig(A - B*Kx) =\n');
disp(eig(A - B*Kx));

% MATLAB balance acts on a matrix, not directly on a state-space realization.
% Use it as a diagnostic for eigenvalue computation, not blindly as a physical
% coordinate transformation for controller implementation.
[Abal, Tbal] = balance(A);
fprintf('cond(Abal) from balance(A) = %.6e\n', cond(Abal));

% Optional balanced-realization workflow if available:
sys = ss(A, B, C, D);
try
    [sysb, g] = balreal(sys);
    fprintf('Hankel singular values from balreal:\n');
    disp(g);
catch ME
    fprintf('balreal unavailable or not applicable: %s\n', ME.message);
end

% Simulink note:
% Use Az, Bz, Cz, D in a State-Space block when the simulated states are z.
% To display physical states, add a Gain block S after the state output.
