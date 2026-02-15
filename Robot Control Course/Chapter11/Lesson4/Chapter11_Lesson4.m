
J = 0.05;
d = 0.01;

A = [0 1 0;
     0 -d/J 0;
     0 0 0];
B = [0; 1/J; 0];
C = [1 0 1];
D = 0;

sys_aug = ss(A, B, C, D);

% PD gains
Kp = 50;
Kd = 2 * sqrt(Kp * J);
K  = [Kp Kd 0];

% Continuous-time closed-loop (assuming perfect estimator)
Acl = A - B * K;
sys_cl = ss(Acl, B, C, D);

% Discretization for implementation (zero-order hold)
Ts = 0.001;
sysd = c2d(sys_aug, Ts, 'zoh');

Ad = sysd.A;
Bd = sysd.B;
Cd = sysd.C;

% Luenberger observer for augmented state (without delay)
L = place(Ad', Cd', [0.1 0.09 0.08]).';

% Simulink hint:
%  - Create a State-Space block implementing the augmented dynamics.
%  - Add a PD controller block using estimated q and qd.
%  - Insert a "Transport Delay" block before the observer input.
%  - Implement the observer using a Discrete State-Space block with matrices:
%       x_hat(k+1) = Ad*x_hat + Bd*u + L*(y_delayed - Cd*x_hat)
%  - Scope joint position, estimated bias, and control effort.
