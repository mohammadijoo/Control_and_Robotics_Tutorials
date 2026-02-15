% Parameters
R = 1.2; L = 2e-3; ke = 0.08; kt = ke;
J = 5e-4; b = 1e-3;

A = [-R/L,   -ke/L;
      kt/J,  -b/J];
B = [1/L; 0];
C = [0 1];   % output speed omega
D = 0;

sys = ss(A,B,C,D);

% PID gains
Kp = 0.4; Ki = 30; Kd = 0;

Cpid = pid(Kp,Ki,Kd);
Tcl = feedback(Cpid*sys, 1);

step(Tcl); grid on; title('Closed-loop speed response')
