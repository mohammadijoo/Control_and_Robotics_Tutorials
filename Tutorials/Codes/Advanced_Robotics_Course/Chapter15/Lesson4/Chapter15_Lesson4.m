function [rmse, x_hist] = swarm_consensus_matlab(N, steps, p_fail, p_drop, eps, noise_std)
    if nargin < 1, N = 50; end
    if nargin < 2, steps = 200; end
    if nargin < 3, p_fail = 0.2; end
    if nargin < 4, p_drop = 0.1; end
    if nargin < 5, eps = 0.2; end
    if nargin < 6, noise_std = 0.01; end

    rng(0);
    x = -1 + 2 * rand(N, 1);
    alive = rand(N, 1) >= p_fail;

    x_hist = zeros(N, steps + 1);
    x_hist(:, 1) = x;

    for k = 1:steps
        x_new = x;
        for i = 1:N
            if ~alive(i), continue; end
            left = mod(i - 2, N) + 1;
            right = mod(i, N) + 1;

            sumDiff = 0;
            count = 0;

            if rand() >= p_drop && alive(left)
                sumDiff = sumDiff + (x(left) - x(i));
                count = count + 1;
            end
            if rand() >= p_drop && alive(right)
                sumDiff = sumDiff + (x(right) - x(i));
                count = count + 1;
            end

            if count > 0
                avgDiff = sumDiff / count;
                x_new(i) = x(i) + eps * avgDiff + noise_std * randn();
            end
        end
        x = x_new;
        x_hist(:, k + 1) = x;
    end

    idx_alive = find(alive);
    if isempty(idx_alive)
        rmse = 0;
        return;
    end
    mean_val = mean(x(idx_alive));
    rmse = sqrt(mean((x(idx_alive) - mean_val).^2));
end
      
