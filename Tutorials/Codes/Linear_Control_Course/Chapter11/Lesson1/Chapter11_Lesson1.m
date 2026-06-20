% Plant parameters
k = 1;
tau = 0.5;

% Plant transfer function G(s) = k / (tau s + 1)
s = tf('s');
G = k / (tau*s + 1);

% Proportional controller C(s) = Kp
Kp = 5;
C = Kp;

% Closed-loop transfer function with unity feedback
T = feedback(C*G, 1);  % T(s) = (C*G) / (1 + C*G)

% Step response
figure;
step(T);
grid on;
title('Closed-loop step response with proportional control');

% Steady-state value and error
[y_ss, t_out] = step(T, 10);
y_final = y_ss(end);
e_ss = 1 - y_final
