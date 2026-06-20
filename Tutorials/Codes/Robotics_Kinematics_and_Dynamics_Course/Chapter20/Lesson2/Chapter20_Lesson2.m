% Example: linearize a rigidBodyTree dynamics around configuration q0
robot = importrobot('my_robot.urdf');
robot.DataFormat = 'row';

q0 = homeConfiguration(robot);
dq0 = zeros(size(q0));
tau0 = zeros(size(q0));

% Build state-space from manipulator equations (symbolic or numeric)
% For illustration, suppose we have functions:
%   [M, C, g] = manipulatorDynamics(robot, q0, dq0);

[M0, C0, g0] = manipulatorDynamics(robot, q0, dq0);
n = numel(q0);

A = [zeros(n) eye(n);
     -M0 \ (d_g_dq(robot, q0))   -M0 \ C0];
B = [zeros(n);
     M0 \ eye(n)];

C = eye(2*n);  % full-state output
D = zeros(2*n, n);

G = ss(A, B, C, D);

% Balanced realization and truncation
[Gb, ~, T, Ti] = balreal(G);   % balanced realization
% Use model order selection tool or Hankel singular values
hsv = hsvd(Gb);
r = find(cumsum(hsv) / sum(hsv) >= 0.99, 1);  % retain 99% energy

Gr = modred(Gb, r+1:2*n, 'truncate');        % reduced-order model

% Simulink: you can use linearize on your Simulink robot model block,
% then apply the same balreal/modred steps to the resulting ss object.
      
