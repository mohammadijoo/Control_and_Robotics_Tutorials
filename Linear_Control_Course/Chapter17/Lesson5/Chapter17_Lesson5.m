% Plant: G(s) = 1 / (s (s + 1))
s = tf('s');
G = 1 / (s * (s + 1));

% Desired crossover
omega_gc = 0.7;
magG = abs(freqresp(G, omega_gc));  % |G(j w)|
K = 1 / magG;
C = K;

L = C * G;

[gm, pm, wgc_num, wpc_num] = margin(L);
fprintf('Designed K = %.4f\n', K);
fprintf('Phase margin = %.2f deg at w = %.3f rad/s\n', pm, wgc_num);

Tcl = feedback(L, 1);
step(Tcl);
grid on;
title('Closed-loop step response, proportional design');

% Simulink note:
% In Simulink, one would implement:
%   - A "Gain" block with value K,
%   - A Transfer Fcn block for 1 / (s (s + 1)),
%   - A Sum block for feedback.
% Then use the Linear Analysis Tool to confirm margins.
