function trust_traj = update_trust(successes, lambda, T0)
% successes: row or column vector of 0/1
% lambda: scalar in (0,1]
% T0: initial trust

if nargin < 3
    T0 = 0.5;
end
if nargin < 2
    lambda = 0.2;
end

T = T0;
N = numel(successes);
trust_traj = zeros(N+1,1);
trust_traj(1) = T;

for k = 1:N
    s = double(successes(k));
    T = (1 - lambda) * T + lambda * s;
    trust_traj(k+1) = T;
end
end

% Example script:
% seq = [1 1 0 1 1 1 0 1 1 1];
% T = update_trust(seq, 0.3, 0.2);
% plot(0:numel(seq), T, '-o');
% xlabel('Step'); ylabel('Trust T_k'); grid on;
      
