function metrics = evaluateTrajectory(q, t)
% Evaluate path length and jerk cost for a joint trajectory.
% q: N-by-n matrix of joint positions.
% t: N-by-1 vector of time stamps (strictly increasing).

dt = diff(t);           % (N-1)-by-1
dq = diff(q) ./ dt;     % (N-1)-by-n
ddq = diff(dq) ./ dt(1:end-1);      % (N-2)-by-n
dddq = diff(ddq) ./ dt(1:end-2);    % (N-3)-by-n

% Discrete path length in configuration space (unit weights).
L = 0.0;
for k = 1:(size(q, 1) - 1)
    diffq = q(k + 1, :) - q(k, :);
    L = L + norm(diffq, 2);
end

% Jerk-based smoothness cost: integral of squared jerk norm.
t_mid = t(3:end-1);           % time grid for jerk samples
jerk_sq = sum(dddq.^2, 2);    % squared jerk norm at each sample
smoothness = trapz(t_mid, jerk_sq);

metrics.pathLength = L;
metrics.jerkCost   = smoothness;
end
      
