% Chapter8_Lesson4.m
% Computing Phi(t) using Jordan form in MATLAB.
% Requires Symbolic Math Toolbox for jordan(); otherwise use the supplied P,J.

clear; clc;

syms t real

J = [2 1 0;
     0 2 0;
     0 0 -1];

P = [1 1 0;
     0 1 1;
     1 0 1];

A = P * J / P;

% Exact symbolic transition matrix from the known Jordan form.
expJ = [exp(2*t), t*exp(2*t), 0;
        0,        exp(2*t),   0;
        0,        0,          exp(-t)];

Phi = simplify(P * expJ / P);

disp('A =');
disp(A);
disp('Phi(t) =');
pretty(Phi);

% Numerical validation against MATLAB expm.
tn = 0.40;
PhiJordan = double(subs(Phi, t, tn));
PhiExpm = expm(double(A) * tn);

disp('Phi(0.40) from Jordan form =');
disp(PhiJordan);
disp('Phi(0.40) from expm(A*t) =');
disp(PhiExpm);
disp('Frobenius error =');
disp(norm(PhiJordan - PhiExpm, 'fro'));

% Simulink connection:
% A Continuous State-Space block with matrix A and initial condition x0 evolves as
% x(t)=Phi(t)x0 when the input is zero. Use Phi above to verify the block output.
