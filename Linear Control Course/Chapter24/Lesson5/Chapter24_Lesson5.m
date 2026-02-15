J_nom = 0.01;   % nominal inertia
B_nom = 0.1;    % viscous friction
K_gain = 1.0;   % motor gain

s = tf('s');
G_nom = K_gain / (J_nom * s + B_nom);

Kc = 2.0;
C = Kc;

L_nom = C * G_nom;
T_nom = feedback(L_nom, 1);

[gm, pm, wcp, wcg] = margin(L_nom);
fprintf('Nominal GM (dB): %.2f\n', 20*log10(gm));
fprintf('Nominal PM (deg): %.2f\n', pm);

% Monte Carlo over uncertain inertia (e.g. payload changes)
N = 200;
J_vals = J_nom * (0.5 + rand(N,1));   % J in [0.5 J_nom, 1.5 J_nom]
isStable = false(N,1);
Ms_vals = zeros(N,1);

for k = 1:N
    J = J_vals(k);
    G = K_gain / (J * s + B_nom);
    L = C * G;
    T = feedback(L, 1);
    cl_poles = pole(T);
    isStable(k) = all(real(cl_poles) < 0);
    S = 1 / (1 + L);
    [magS, ~] = bode(S);
    Ms_vals(k) = max(magS(:));
end

fprintf('Stable trials: %d / %d\n', sum(isStable), N);
fprintf('Max sensitivity peak over trials: %.2f\n', max(Ms_vals));

% Optional: visualize distribution of M_S
figure; histogram(Ms_vals, 20);
xlabel('M_S'); ylabel('count'); grid on;
title('Distribution of sensitivity peak over uncertain inertia');
