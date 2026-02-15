% Example plant and controller (students know linear control)
s = tf('s');
G = 1/(s*(0.5*s+1));     % simple second-order-like plant
K = 5*(s+2)/(s+10);      % lead/lag-ish controller

L = series(K,G);
[GM, PM, wgc, wpc] = margin(L);

Td_max = PM*pi/180 / wgc;  % Td < PM / wgc (Section 3)
fprintf('Phase margin = %.2f deg, wgc = %.2f rad/s, Td_max = %.3f s\n', PM, wgc, Td_max);

% In Simulink:
% - Use Transfer Fcn blocks for G and K
% - Insert a Transport Delay block of value Td
% - Sweep Td to verify closed-loop stability boundary.
