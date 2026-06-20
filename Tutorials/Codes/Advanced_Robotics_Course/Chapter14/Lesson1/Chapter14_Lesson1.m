function multi_robot_rendezvous
    N = 5;
    dt = 0.05;
    T  = 5.0;
    steps = round(T / dt);
    k_gain = 1.0;

    rng(1);
    p0 = -3 + 6 * rand(N,1);

    % ring neighbors
    neighbors = cell(N,1);
    for i = 1:N
        neighbors{i} = [mod(i-2,N)+1, mod(i,N)+1]; % 1-based ring
    end

    p_central = p0;
    p_decent  = p0;

    p_hist_central = zeros(N, steps+1);
    p_hist_decent  = zeros(N, steps+1);
    p_hist_central(:,1) = p_central;
    p_hist_decent(:,1)  = p_decent;

    for k = 1:steps
        p_central = centralized_step(p_central, p0, k_gain, dt);
        p_decent  = decentralized_step(p_decent, neighbors, k_gain, dt);
        p_hist_central(:,k+1) = p_central;
        p_hist_decent(:,k+1)  = p_decent;
    end

    t = 0:dt:T;
    figure;
    subplot(2,1,1);
    plot(t, p_hist_central);
    title('Centralized rendezvous');
    xlabel('time'); ylabel('position');

    subplot(2,1,2);
    plot(t, p_hist_decent);
    title('Decentralized rendezvous');
    xlabel('time'); ylabel('position');
end

function p_next = centralized_step(p, p0, k_gain, dt)
    N = numel(p);
    p_avg0 = mean(p0);
    u = -k_gain * (p - p_avg0);
    p_next = p + dt * u;
end

function p_next = decentralized_step(p, neighbors, k_gain, dt)
    N = numel(p);
    u = zeros(N,1);
    for i = 1:N
        s = 0.0;
        for j = neighbors{i}
            s = s + (p(i) - p(j));
        end
        u(i) = -k_gain * s;
    end
    p_next = p + dt * u;
end
      
