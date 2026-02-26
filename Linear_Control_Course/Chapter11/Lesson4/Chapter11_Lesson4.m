% First-order plant G(s) = K / (tau s + 1)
K   = 2.0;
tau = 0.5;
s   = tf('s');
G   = K / (tau * s + 1);

% Parallel-form PID parameters
Kp = 8.0;
Ki = 5.0;
Kd = 0.1;

% MATLAB PID object (parallel structure)
C_pid = pid(Kp, Ki, Kd);

% Closed-loop transfer function with unity feedback
T = feedback(C_pid * G, 1);

figure;
step(T);
title('Step response with parallel PID controller');

% In Simulink, the same structure is realized by connecting:
%  - a "PID Controller" block (set to Parallel form),
%  - the plant G(s) implemented as a Transfer Fcn or State-Space block,
%  - and a unity feedback loop with appropriate summation blocks.
