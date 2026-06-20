% Simple loop: L(s) = K / (s (s + 1))
K = 1;
s = tf('s');
P = 1/(s*(s + 1));
C = K;
L = C*P;

% Compute classical gain and phase margins
[Gm, Pm, Wcg, Wcp] = margin(L);
fprintf('Gain margin (abs): %g\n', Gm);
fprintf('Gain margin (dB): %g\n', 20*log10(Gm));
fprintf('Phase margin (deg): %g\n', Pm);

figure;
margin(L); grid on;
title('Loop margins for L(s) = K/(s(s+1))');

% Robotics context: linearizing a joint from a rigid-body tree
% (requires Robotics System Toolbox)
% robot = importrobot('example.urdf');
% jointIdx = 1;
% [A,B,Cm,Dm] = linearize(robot, 'someConfig', 'someIO');
% P_joint = ss(A,B,Cm,Dm);        % joint LTI model
% C_joint = pid(10, 1, 0.1);      % example PID
% L_joint = C_joint*P_joint;
% margin(L_joint);
