
J = 0.01;
b = 0.1;
A = [0 1;
     0 -b/J];
B = [0;
     1/J];
C = eye(2);
D = zeros(2,1);

Ts = 1e-3;

sysc = ss(A, B, C, D);
% Exact ZOH discretization
sysd = c2d(sysc, Ts, 'zoh');

Phi   = sysd.A;
Gamma = sysd.B;

disp('Phi =');   disp(Phi);
disp('Gamma ='); disp(Gamma);

% Simulink implementation:
%  - Use a "State-Space" block with A=Phi, B=Gamma, C=eye(2), D=zeros(2,1).
%  - Drive the block with discrete torque command u[k] from a "Zero-Order Hold".
%  - Set the Simulink solver to "discrete (no continuous states)" with step Ts.
