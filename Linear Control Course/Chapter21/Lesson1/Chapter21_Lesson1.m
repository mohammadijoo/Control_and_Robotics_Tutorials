% DC motor parameters
J = 0.01;
b = 0.1;
K = 1.0;

s = tf('s');

% Plant and PI controller
G = K / (J*s^2 + b*s);
Kp = 20;
Ki = 40;
C = Kp + Ki/s;

L = C*G;
T = feedback(L, 1);

% Bode plot of loop transfer
figure; bode(L); grid on;

% Step response of closed loop
figure; step(T); grid on;

% In Simulink, the same servo can be implemented using blocks:
%   - Transfer Fcn for G(s)
%   - Sum blocks for feedback
%   - PI controller block or manual gain + integrator
% This allows visual experimentation with loop shapes and performance.
