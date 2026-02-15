% Example: Y(s) = 1 / (s*(s+3))
num = 1;
den = [1 3 0];  % s^2 + 3s + 0

% Partial fraction expansion
[R, P, K] = residue(num, den);  % R: residues, P: poles, K: direct term

disp('Residues R:');
disp(R);
disp('Poles P:');
disp(P);

% Build transfer function and simulate step response
sys = tf(num, den);  % requires Control System Toolbox
figure;
step(sys);
title('Step response of Y(s) = 1 / (s (s + 3))');

% In Simulink:
% - Use a Step block as input, connect to a Transfer Fcn block with numerator [1]
%   and denominator [1 3 0].
% - Scope block displays y(t).
% Such blocks are often used inside larger robotic models built with Robotics System Toolbox.
