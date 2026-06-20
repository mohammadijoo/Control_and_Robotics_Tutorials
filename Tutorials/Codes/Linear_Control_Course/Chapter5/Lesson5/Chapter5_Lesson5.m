% Exact and approximate transfer functions
s = tf('s');
G_full = 1 / ((1 + s) * (1 + 0.1*s));
G_1st  = 1 / (1 + s);

figure;
step(G_full, G_1st);
legend('Full model', '1st-order approx');
grid on;

% In Simulink:
% - Place two Transfer Fcn blocks (G_full and G_1st)
% - Drive them with the same Step block
% - Compare outputs on a Scope block
