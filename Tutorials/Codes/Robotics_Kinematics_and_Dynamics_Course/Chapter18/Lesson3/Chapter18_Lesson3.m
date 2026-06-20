function [jerk, J] = jerk_smoothness(Q, dt, W)
% Q: (N+1)-by-nJoints matrix of joint positions.
% dt: sampling period.
% W:  nJoints-by-nJoints weighting matrix (optional, defaults to eye).

[Nplus1, nJoints] = size(Q);
if Nplus1 < 4
    error('Need at least 4 samples');
end
N = Nplus1 - 1;
jerk = zeros(N-2, nJoints);

if nargin < 3
    W = eye(nJoints);
end

dt3 = dt^3;

for k = 2:(N-1)
    jerk(k-1,:) = ( Q(k+2,:) - 3*Q(k+1,:) + 3*Q(k,:) - Q(k-1,:) ) / dt3;
end

J = 0.0;
for k = 1:(N-2)
    j_k = jerk(k,:).';
    J = J + j_k.' * W * j_k;
end
J = J * dt;
end

% Example usage:
T = 2.0;
dt = 0.01;
t = 0:dt:T;
s = t / T;
q = 10*s.^3 - 15*s.^4 + 6*s.^5;   % 1-DOF smooth trajectory
Q = q(:);                          % column vector (N+1)-by-1

[jerk, J] = jerk_smoothness(Q, dt);
fprintf('Smoothness index J = %.6f\n', J);
      
