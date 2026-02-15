% Parameters
J = 0.01;
b = 0.1;
Km = 1.0;

s = tf('s');
G = Km / (s * (J * s + b));   % Plant

% 1) Root locus
figure;
rlocus(G);
grid on;
title('Root locus of G(s)');

% Choose gain K from root locus (e.g., by clicking or inspection)
K_des = 50;
L = K_des * G;                % Loop transfer function
T = feedback(L, 1);           % Closed-loop transfer function

% 2) Bode and margins
figure;
margin(L);                    % Bode plot with gain/phase margins
grid on;

% 3) Nyquist plot
figure;
nyquist(L);
grid on;

% Step response to verify time-domain specs
figure;
step(T);
grid on;
title('Closed-loop step response');
