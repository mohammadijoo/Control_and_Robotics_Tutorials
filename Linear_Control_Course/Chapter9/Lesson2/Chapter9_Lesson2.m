% Open-loop transfer function: G(s) = K (s+1) / [s (s+2) (s+4)]
num = [1 1];                 % s + 1
den = conv([1 0], conv([1 2], [1 4]));  % s * (s+2) * (s+4)

G = tf(num, den);

figure;
rlocus(G)
title('Root locus of G(s) = K (s+1) / [s (s+2) (s+4)]')
grid on

% Example: choose a gain K for desired closed-loop pole location
s_des = -2 + 2j;  % desired closed-loop pole
[K, poles] = rlocfind(G, s_des);  %#ok<NOPTS>
disp('Selected gain K:');
disp(K);
disp('Closed-loop poles at this K:');
disp(poles);

% In a Simulink model, G(s) can be implemented with Transfer Fcn blocks
% and the gain K with a Gain block in the forward path. The root locus
% analysis can be performed analytically using this G and then K applied
% in the Simulink block diagram for robotic joint or actuator models.
