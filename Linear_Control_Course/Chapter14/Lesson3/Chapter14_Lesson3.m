% Parameters
K = 10;
z = 10;
wn = 20;
zeta = 0.3;
Tdelay = 0.02;

s = tf('s');

% Real zero and complex pole pair
G_zero = 1 + s/z;
G_poles = 1 / (1 + 2*zeta*(s/wn) + (s/wn)^2);

% Exact delay in MATLAB (handled symbolically by control functions)
G_delay = exp(-Tdelay*s);

% Complete plant
G_exact = K * G_zero * G_poles * G_delay;

% For some algorithms, use Pade approximation:
[numd, dend] = pade(Tdelay, 1);
G_pade = K * G_zero * G_poles * tf(numd, dend);

% Bode plots
omega = logspace(0, 3, 500);
figure;
bode(G_exact, omega); grid on;
title('Bode of G(s) with exact delay');

figure;
bode(G_pade, omega); grid on;
title('Bode of G(s) with Pade-approximated delay');

% Simulink:
%  - Use "Transfer Fcn" blocks for the zero/pole part.
%  - Use "Transport Delay" to represent e^{-sT}.
%  - Use "Bode Plot" (Control System Toolbox) for frequency-domain analysis.
