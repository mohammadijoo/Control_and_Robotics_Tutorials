g = 9.81; Jtheta = 0.02;

A = [0 1 0 0;
     0 0 g 0;
     0 0 0 1;
     0 0 0 0];
B = [0;0;0;1/Jtheta];

Q = diag([10, 2, 50, 5]);  % penalize position+attitude
R = 0.1;                   % control effort
K = lqr(A,B,Q,R);

Acl = A - B*K;
sys_cl = ss(Acl, B, eye(4), 0);
step(sys_cl(1,1)); grid on;
title('Closed-loop step response for x with LQR hover control');
      