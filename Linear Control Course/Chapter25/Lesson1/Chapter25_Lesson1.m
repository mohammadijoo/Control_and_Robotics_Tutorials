% Inner and outer plants
s = tf('s');
G1 = 5 / (0.1 * s + 1);
G2 = 2 / (0.5 * s + 1);

% Controllers
k1 = 8;  % inner
k2 = 1.5; % outer
C1 = k1;
C2 = k2;

% Inner closed loop
Gcl1 = feedback(C1 * G1, 1);

% Equivalent outer plant and cascade closed loop
Geq = Gcl1 * G2;
T_cascade = feedback(C2 * Geq, 1);

% Single-loop for comparison
G_single = G1 * G2;
C_single = k1 * k2;
T_single = feedback(C_single * G_single, 1);

% Step responses
figure;
step(T_cascade, T_single);
legend('Cascade', 'Single-loop');
title('Cascade vs Single-loop Step Response');
grid on;
