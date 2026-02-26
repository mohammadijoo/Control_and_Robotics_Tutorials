
% Joint PD closed-loop example
J_nom = 0.5;
Kp = 50;
Kd = 5;

A = [0 1;
    -Kp/J_nom -Kd/J_nom];
B_d = [0; 1/J_nom];
C = eye(2);
D = [0; 0];

% Eigenvalues and damping
ev = eig(A);
alpha = max(real(ev));

fprintf('Eigenvalues:\n');
disp(ev);
fprintf('Spectral abscissa alpha(A_cl) = %g\n', alpha);

% Lyapunov equation
Q = eye(2);
P = lyap(A', Q);
lam_min_Q = min(eig(Q));
P_norm = norm(P, 2);
delta_bar = lam_min_Q / (2 * P_norm);
fprintf('Conservative bound on ||Delta A||_2: %g\n', delta_bar);

% Input-output robustness (H-infinity norm)
G = ss(A, B_d, C, D);
[hinfNorm, freq] = hinfnorm(G);
fprintf('H-infinity norm: %g at frequency %g rad/s\n', hinfNorm, freq);

% Simulink integration:
% 1. Build or reuse a Simulink model of your robot + controller.
% 2. Use linearize() or slLinearizer to obtain (A,B,C,D) at operating points.
% 3. Call eig, lyap, hinfnorm, and margin on these linearizations.
