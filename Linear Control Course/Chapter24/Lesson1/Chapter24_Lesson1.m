J  = 0.01;
b  = 0.1;
Kt = 0.5;

Kp = 20;
Ki = 10;

% Plant and controller
G = tf(Kt, [J b 0]);          % G(s) = Kt / (J s^2 + b s)
C = tf([Kp Ki], [1 0]);       % C(s) = (Kp s + Ki) / s
L = series(C, G);
T = feedback(L, 1);

% Classical stability margins
[gm, pm, w_gc, w_pc] = margin(L);
gm_db = 20*log10(gm);

fprintf("Gain margin: %.3f (%.2f dB) at w_pc = %.3f rad/s\n", gm, gm_db, w_pc);
fprintf("Phase margin: %.3f deg at w_gc = %.3f rad/s\n", pm, w_gc);

% Delay and multiplicative uncertainty margins
pm_rad   = deg2rad(pm);
tau_max  = pm_rad / w_gc;
deltaMax = 2 * sin(0.5 * pm_rad);
fprintf("Approx delay margin tau_max = %.4f s\n", tau_max);
fprintf("Approx multiplicative uncertainty delta_max = %.3f\n", deltaMax);

% Bode and Nyquist plots
figure;
margin(L); grid on;           % Bode with margins highlighted

figure;
nyquist(L); grid on;

% Simulink integration (example sketch):
% 1. Build a Simulink model with 'Plant', 'Controller', and 'Feedback' blocks.
% 2. Use 'linearize' or 'linmod' to obtain the linearized open-loop model L.
% 3. Use 'margin' on the linearized L to verify GM/PM even when the plant
%    is part of a more complex robot model (e.g., using Robotics System Toolbox).
