% Plant: G(s) = 1 / (s (s + 2))
s = tf('s');
G = 1 / (s * (s + 2));

% Lag compensator parameters
Kc = 5;
omega_p = 0.2;
omega_z = 1.0;

Clag = Kc * (s/omega_z + 1) / (s/omega_p + 1);

L = series(Clag, G);
T = feedback(L, 1);

figure;
bode(L); grid on; title('Open-loop with phase lag compensator');

% Steady-state velocity constant Kv
sG = series(Clag, G * s);   % s * L(s)
Kv = dcgain(sG);

disp(['Approximate Kv = ', num2str(Kv)]);

% Simulink note:
% In Simulink, use:
%  - "Transfer Fcn" block for G(s)
%  - "Transfer Fcn" block for Clag(s)
%  - "Sum" block for feedback
%  - Optional: "Robot" plant from Robotics System Toolbox linearization
