% Plant: G(s) = 1 / (s (s + 2))
s = tf('s');
G = 1 / (s * (s + 2));

% Root locus plot
figure;
rlocus(G);
title('Root locus of G(s) = 1 / (s (s + 2))');

% Closed-loop transfer function for a specific gain K
K = 5;
T = feedback(K * G, 1);   % Unity feedback
pole(T)

% In Simulink:
% - Use a Transfer Fcn block for G(s)
% - A Gain block for K
% - Sum block for negative feedback
% - Use the "Control System Designer" or "Root Locus" app to interactively move poles.
