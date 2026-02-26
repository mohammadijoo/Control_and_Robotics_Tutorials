function [s, sdot, t] = topp1D(s_f, N, v_max, a_max)
    % Time-parameterization for 1D path q(s) = s
    s = linspace(0, s_f, N + 1);
    ds = diff(s);

    sdot_max = v_max * ones(1, N + 1);

    % Forward pass
    sdot = zeros(1, N + 1);
    for k = 1:N
        sdot_cand_sq = sdot(k)^2 + 2 * a_max * ds(k);
        sdot_cand = sqrt(max(0, sdot_cand_sq));
        sdot(k + 1) = min(sdot_cand, sdot_max(k + 1));
    end

    % Backward pass
    for k = N:-1:1
        sdot_cand_sq = sdot(k + 1)^2 + 2 * a_max * ds(k);
        sdot_cand = sqrt(max(0, sdot_cand_sq));
        sdot(k) = min([sdot(k), sdot_cand, sdot_max(k)]);
    end

    % Time stamps
    t = zeros(1, N + 1);
    for k = 1:N
        denom = sdot(k) + sdot(k + 1);
        if denom <= 1e-9
            error("Infeasible profile: zero velocity segment");
        end
        t(k + 1) = t(k) + 2 * ds(k) / denom;
    end
end

% Example usage:
% [s, sdot, t] = topp1D(1.0, 100, 1.0, 2.0);
% q = s; % since q(s) = s in 1D
% % Create a timeseries for Simulink:
% q_ts = timeseries(q, t);
% sdot_ts = timeseries(sdot, t);
% % These time series can be fed into a "From Workspace" block in Simulink.
      
