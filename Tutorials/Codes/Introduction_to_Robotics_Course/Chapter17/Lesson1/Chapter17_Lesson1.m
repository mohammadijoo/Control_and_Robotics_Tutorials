function stats = simulate_workcell(K, lam)
    if nargin < 1, K = 1000; end
    if nargin < 2, lam = 0.7; end

    q = 0;
    total_departures = 0;
    total_q = 0;

    for k = 1:K
        arrivals = rand() < lam;  % 0 or 1
        q = q + arrivals;

        if q > 0
            q = q - 1;
            total_departures = total_departures + 1;
        end

        total_q = total_q + q;
    end

    stats.avg_queue_length = total_q / K;
    stats.throughput = total_departures / K;
    stats.utilization = stats.throughput; % capacity = 1 job/step
end

% Example usage:
% s = simulate_workcell(10000, 0.7);
% disp(s);
      
