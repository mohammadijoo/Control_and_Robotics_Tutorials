function collective_transport_demo()
    N  = 6;
    kp = 2.0; kv = 0.5;
    alpha = 1.0; beta = 1.0;
    m  = 10.0;
    dt = 0.01;
    steps = 1000;

    c = [0; 0];
    v = [0; 0];
    goal = [1; 0];
    f = ones(N, 1) / N;
    L = ringLaplacian(N);

    for k = 1:steps
        e = c - goal;
        Fd = -kp * e - kv * v;
        normFd = norm(Fd);
        if normFd < 1e-6
            break;
        end
        d = Fd / normFd;
        Fd_mag = normFd;

        g = f - (Fd_mag / N) * ones(N, 1);
        gdot = -alpha * (L * g) - beta * g;
        f = f + dt * gdot;
        f = max(0.0, min(5.0, f));

        F_net = d * sum(f);
        a = F_net / m;
        v = v + dt * a;
        c = c + dt * v;
    end

    disp('Final position:');
    disp(c);
end

function L = ringLaplacian(N)
    L = zeros(N, N);
    for i = 1:N
        L(i, i) = 2.0;
        L(i, mod(i - 2, N) + 1) = -1.0;
        L(i, mod(i, N) + 1) = -1.0;
    end
end
      
