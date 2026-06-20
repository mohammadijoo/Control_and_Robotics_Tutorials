% Plant and inner loop
s = tf('s');
Gp = 1 / (s + 1);
k_i = 9;
Ci = k_i;
Gi_cl = feedback(Ci * Gp, 1);    % inner closed loop

% Outer PI loop
k_p = 0.5;
k_I = 1.0;
Co = k_p + k_I / s;

L_outer = Co * Gi_cl;
T_outer = feedback(L_outer, 1);

figure;
step(T_outer);
title('Outer-loop step response with inner loop closed');
grid on;
