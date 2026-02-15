% Parameters
M = 2.0; A_p = 3e-4; A_r = 3e-4;
V_A0 = 2e-5; V_B0 = 2e-5;
k = 1.2; R = 287; T = 293;
b = 15; pA0 = 101325; pB0 = 101325;

KqA = 1.3e-4; KpA = 2.0e-9;
KqB = 1.3e-4; KpB = 2.0e-9;

% Linearized matrices
A = [ 0 1 0 0;
      0 -b/M A_p/M -A_r/M;
     -k*A_p*pA0/(V_A0) 0 -(k*R*T/V_A0)*KpA 0;
      k*A_r*pB0/(V_B0) 0 0 -(k*R*T/V_B0)*KpB ];

B = [0; 0; (k*R*T/V_A0)*KqA; (k*R*T/V_B0)*KqB ];

C = eye(4); D = zeros(4,1);
sys = ss(A,B,C,D);

t = 0:1e-3:0.4;
u = double(t >= 0.05); % step
y = lsim(sys, u, t);

plot(t, y(:,1)); grid on;
xlabel('Time (s)'); ylabel('x (m)');
title('Linearized Pneumatic Axis Step Response');
