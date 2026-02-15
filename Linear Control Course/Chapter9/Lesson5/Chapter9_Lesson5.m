% Plant: G(s) = K / (s (s+2) (s+5))
s = tf('s');
G = 1 / (s * (s + 2) * (s + 5));

figure;
rlocus(G);
title('Root locus of G(s) = K / (s (s+2) (s+5))');
grid on;

% Read off gain for a chosen dominant pole location, or use rlocfind
[k_selected, poles_selected] = rlocfind(G);
disp(k_selected);
disp(poles_selected);
