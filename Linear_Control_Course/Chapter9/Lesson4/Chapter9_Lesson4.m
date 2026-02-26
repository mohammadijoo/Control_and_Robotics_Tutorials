% Define transfer function G(s) = 1 / (s (s + 2) (s + 4))
s = tf('s');
G = 1 / (s * (s + 2) * (s + 4));

% Root locus of K G(s)
figure;
rlocus(G);
title('Root locus of K / (s (s+2) (s+4))');

% Analytical critical gain from Routh
Kcrit = 48;

% Closed-loop transfer function for Kcrit
K = Kcrit;
Tcrit = feedback(K * G, 1);
poles_crit = pole(Tcrit)

% Check that poles lie on imaginary axis
damp(Tcrit)

% In Simulink:
% - Create a plant block implementing 1 / (s (s+2) (s+4)).
% - Connect a Gain block K, and a unity feedback loop.
% - Use a mask parameter for K and vary it up to Kcrit to observe transition
%   from overdamped to sustained oscillations at K = Kcrit.
