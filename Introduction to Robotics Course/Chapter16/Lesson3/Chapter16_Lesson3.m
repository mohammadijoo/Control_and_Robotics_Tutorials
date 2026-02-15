J  = 0.02;
B  = 0.01;
N  = 10.0;
Kt = 0.05;
Ke = 0.05;
R  = 2.0;

Kp = 32.0;
Kd = 1.7;

Beq = B + (N^2)*Kt*Ke/R;
Ka  = N*Kt/R;

s = tf('s');

% Plant from voltage to angle: q(s) / v(s)
Gv = Ka / (J*s^2 + Beq*s);

% PD controller
C = Kp + Kd*s;

% Closed-loop from q_ref to q
T = feedback(C*Gv, 1);

step(0.5*T);                % 0.5 rad step
grid on;
title('Joint step response (co-designed)');

info = stepinfo(0.5*T);     % settling time, overshoot, etc.
disp(info);
      
