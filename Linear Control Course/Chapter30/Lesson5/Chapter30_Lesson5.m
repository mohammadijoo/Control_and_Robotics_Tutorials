m = 1.0;
b = 0.5;
k = 2.0;

A = [0 1;
    -k/m -b/m];
B = [0;
     1/m];
C = [1 0];
D = 0;

% LQR weights
Q = diag([10 1]);
R = 1;

% Continuous-time LQR
[K, P, e_cl] = lqr(A, B, Q, R);

% Discretize for digital implementation
Ts = 0.01;
sysc = ss(A, B, C, D);
sysd = c2d(sysc, Ts, 'zoh');
Ad = sysd.A;
Bd = sysd.B;

% In Simulink, you would implement:
%  - a State-Space block with Ad,Bd,C,D
%  - a Discrete-Time State-Space or integrator structure
%  - a Gain block implementing -K
