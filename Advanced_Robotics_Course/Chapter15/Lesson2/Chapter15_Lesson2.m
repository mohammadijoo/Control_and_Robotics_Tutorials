function swarm_sim(behavior)
% behavior: 'aggregate' or 'disperse'

if nargin < 1, behavior = 'aggregate'; end

N = 40;
dt = 0.05;
T = 20.0;
K = round(T / dt);
R = 0.4;
k_agg = 1.0;
k_disp = 0.5;
d_des = 0.2;

x = rand(N, 2);  % positions

for k = 1:K
    u = zeros(N, 2);
    for i = 1:N
        diffs = x - x(i, :);
        dists = sqrt(sum(diffs.^2, 2));
        mask = (dists > 0) & (dists < R);
        nbr_diffs = diffs(mask, :);
        nbr_dists = dists(mask);
        if isempty(nbr_dists), continue; end
        switch behavior
            case 'aggregate'
                u(i, :) = -k_agg * sum(nbr_diffs, 1);
            case 'disperse'
                r2 = nbr_dists .^ 2;
                phi_prime = (r2 - d_des^2) .* nbr_dists;
                dirs = nbr_diffs ./ nbr_dists;
                force = -sum(phi_prime .* dirs, 1);
                u(i, :) = k_disp * force;
        end
    end
    x = x + dt * u;
    if mod(k, 10) == 0
        clf;
        scatter(x(:, 1), x(:, 2), 'filled');
        axis([0 1 0 1]); axis equal;
        title(sprintf('k = %d', k));
        drawnow;
    end
end
end
      
