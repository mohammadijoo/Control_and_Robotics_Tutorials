% Inner plant parameters
L = 2e-3;   % H
R = 0.5;    % Ohm
Ku = 1.0;   % gain

% Desired inner closed-loop specs
zeta_i = 0.7;
omega_n_i = 500;   % rad/s

% PI design
kp = (2 * L * zeta_i * omega_n_i - R) / Ku;
ki = (L * omega_n_i^2) / Ku;

% Transfer functions
s = tf('s');
Gi = Ku / (L * s + R);
Ci = kp + ki / s;

Li = Ci * Gi;
Ti = feedback(Li, 1);
Si = 1 - Ti;

% Step response
figure;
step(Ti);
grid on;
title('Inner current loop step response');

% Bode and margins
figure;
margin(Li);
grid on;
title('Inner loop Bode plot and margins');

disp('Inner PI gains:');
disp(table(kp, ki));
