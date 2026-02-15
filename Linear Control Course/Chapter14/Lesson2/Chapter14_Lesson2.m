% First-order pole and zero
omega_c = 10;   % rad/s
omega_z = 5;    % rad/s

s = tf('s');
G_p = 1 / (1 + s/omega_c);   % first-order pole
G_z = 1 + s/omega_z;         % first-order zero

% Second-order factor (e.g. joint speed dynamics)
omega_n = 20;
zeta = 0.4;
G_2 = omega_n^2 / (s^2 + 2*zeta*omega_n*s + omega_n^2);

G_total = G_p * G_2;

w = logspace(-1, 3, 500);

figure;
bode(G_p, w);
grid on;
title('First-order pole');

figure;
bode(G_2, w);
grid on;
title('Second-order factor');

figure;
bode(G_total, w);
grid on;
title('Combined first- and second-order plant');

% In Simulink, one can linearize a robot joint model and call bode on the
% resulting linearized plant to inspect first- and second-order behavior.
