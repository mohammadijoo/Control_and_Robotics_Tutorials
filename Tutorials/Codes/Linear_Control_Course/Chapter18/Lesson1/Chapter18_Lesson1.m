s = tf('s');

% Plant and controller
P = 1 / (s * (s + 1));  % robot joint approximation
K = 50;
C = K;

% Loop and closed-loop transfer functions
L = C * P;              % loop transfer L(s)
Tref = L / (1 + L);     % Y/R = L/(1+L)
Terr = 1 / (1 + L);     % E/R = 1/(1+L)

% Bode plots of loop and error transfer
w = logspace(-2, 2, 400);
figure;
bode(L, w), grid on;
title('Loop transfer L(s)');

figure;
bode(Terr, w), grid on;
title('Error transfer E/R = 1 / (1 + L)');

% Evaluate error magnitude at w = 1 rad/s
[Terr_mag, Terr_phase] = bode(Terr, 1);
fprintf('Error amplitude ratio at w = 1 rad/s: %g\n', Terr_mag);
