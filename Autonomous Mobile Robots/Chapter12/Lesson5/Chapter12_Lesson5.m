% Chapter12_Lesson5.m
% Lab: Build and Optimize a 2D Pose Graph (SE(2)) using Gauss-Newton / LM
%
% Requirements:
%   - MATLAB base (sparse matrices)
% Optional:
%   - Robotics System Toolbox (not required here)
%
% Run:
%   Chapter12_Lesson5

function Chapter12_Lesson5()
    rng(4);

    N = 60;
    [x0, edges, gt] = simulate_pose_graph(N);

    fprintf('Initial RMSE: pos=%.3f m, ang=%.3f rad\n', rmse_pose(x0, gt));

    x = solve_pose_graph(x0, edges, N, 25, 1e-7, true);

    fprintf('Optimized RMSE: pos=%.3f m, ang=%.3f rad\n', rmse_pose(x, gt));

    % Export to CSV for plotting elsewhere
    out = [reshape(x,3,[])' reshape(gt,3,[])'];
    writematrix(out, "Chapter12_Lesson5_poses.csv");
    disp("Wrote Chapter12_Lesson5_poses.csv");

    % Optional Simulink skeleton (programmatically creates a model)
    % create_pose_graph_simulink_model();
end

% -------------------- SE(2) helpers --------------------

function a = wrap_angle(a)
    a = mod(a + pi, 2*pi) - pi;
    if a <= -pi, a = a + 2*pi; end
end

function R = rot2(th)
    R = [cos(th) -sin(th); sin(th) cos(th)];
end

function xout = se2_compose(x, y)
    R = rot2(x(3));
    t = x(1:2) + R * y(1:2);
    xout = [t(1); t(2); wrap_angle(x(3) + y(3))];
end

function xinv = se2_inverse(x)
    R = rot2(x(3));
    tinv = -(R') * x(1:2);
    xinv = [tinv(1); tinv(2); wrap_angle(-x(3))];
end

function z = se2_between(xi, xj)
    z = se2_compose(se2_inverse(xi), xj);
end

% -------------------- Pose-graph model --------------------

function cost = pose_graph_cost(x, edges)
    cost = 0;
    for k = 1:numel(edges)
        e = edges(k);
        xi = x(3*e.i+1:3*e.i+3);
        xj = x(3*e.j+1:3*e.j+3);
        zhat = se2_between(xi, xj);
        err = se2_compose(se2_inverse(e.z), zhat);
        err(3) = wrap_angle(err(3));
        cost = cost + err' * e.Omega * err;
    end
end

function [err, A, B] = linearize_edge(xi, xj, z)
    % Same model as the Python implementation:
    % e_t = Rz^T (Ri^T (tj-ti) - z_t)
    % e_th = wrap( (thj-thi) - z_th )

    ti = xi(1:2); tj = xj(1:2);
    thi = xi(3);  thj = xj(3);

    Ri = rot2(thi);
    Rz = rot2(z(3));

    dt = tj - ti;
    zhat_t = Ri' * dt;
    zhat_th = wrap_angle(thj - thi);

    e_t = Rz' * (zhat_t - z(1:2));
    e_th = wrap_angle(zhat_th - z(3));
    err = [e_t; e_th];

    A = zeros(3,3); B = zeros(3,3);
    S = [0 -1; 1 0];

    A(1:2,1:2) = -Rz' * Ri';
    B(1:2,1:2) =  Rz' * Ri';

    d_ri = -Ri' * (S * dt);
    A(1:2,3) = Rz' * d_ri;

    A(3,3) = -1;
    B(3,3) =  1;
end

function [H, b] = build_normal_equations(x, edges, N, lambda)
    dim = 3*N;
    H = spalloc(dim, dim, 60*dim); % heuristic nnz
    b = zeros(dim,1);

    for k = 1:numel(edges)
        ed = edges(k);
        i = ed.i; j = ed.j;

        xi = x(3*i+1:3*i+3);
        xj = x(3*j+1:3*j+3);

        [err, A, B] = linearize_edge(xi, xj, ed.z);
        Om = ed.Omega;

        ii = (3*i+1):(3*i+3);
        jj = (3*j+1):(3*j+3);

        H(ii,ii) = H(ii,ii) + A' * Om * A;
        H(ii,jj) = H(ii,jj) + A' * Om * B;
        H(jj,ii) = H(jj,ii) + B' * Om * A;
        H(jj,jj) = H(jj,jj) + B' * Om * B;

        b(ii) = b(ii) + A' * Om * err;
        b(jj) = b(jj) + B' * Om * err;
    end

    if lambda > 0
        d = diag(H);
        H = H + spdiags(lambda*(d + 1e-12), 0, dim, dim);
    end

    % Gauge fix: anchor node 0 by forcing dx0 = 0 (set rows/cols to identity)
    for k = 1:3
        idx = k;
        H(idx,:) = 0; H(:,idx) = 0;
        H(idx,idx) = 1;
        b(idx) = 0;
    end
end

function x = solve_pose_graph(x0, edges, N, maxIters, tol, useLM)
    x = x0;
    lambda = 1e-3;
    cost = pose_graph_cost(x, edges);
    fprintf('iter %d: cost=%.6f\n', 0, cost);

    for it = 1:maxIters
        [H, b] = build_normal_equations(x, edges, N, (useLM * lambda));
        dx = - (H \ b);
        step = norm(dx);

        if step < tol
            fprintf('iter %d: step %.3e < tol; stop.\n', it, step);
            break;
        end

        xNew = x + dx;
        for n=0:(N-1)
            xNew(3*n+3) = wrap_angle(xNew(3*n+3));
        end

        newCost = pose_graph_cost(xNew, edges);

        if ~useLM
            x = xNew; cost = newCost;
            fprintf('iter %d: cost=%.6f step=%.3e\n', it, cost, step);
            continue;
        end

        if newCost < cost
            x = xNew; cost = newCost;
            lambda = max(lambda/3, 1e-9);
            fprintf('iter %d: cost=%.6f step=%.3e lambda=%.2e (accepted)\n', it, cost, step, lambda);
        else
            lambda = min(lambda*5, 1e9);
            fprintf('iter %d: rejected lambda=%.2e\n', it, lambda);
        end
    end
end

% -------------------- Data simulation --------------------

function Om = make_information(sig_xy, sig_th)
    Cov = diag([sig_xy^2, sig_xy^2, sig_th^2]);
    Om = inv(Cov);
end

function [x0, edges, gt] = simulate_pose_graph(N)
    gt = zeros(3*N,1);
    x = [0;0;0];
    for i=0:(N-1)
        gt(3*i+1:3*i+3) = x;
        u = [0.5; 0.0; 0.03];
        x = se2_compose(x, u);
    end

    edges = struct('i',{},'j',{},'z',{},'Omega',{});

    OmOdo = make_information(0.05, 0.02);
    for i=0:(N-2)
        xi = gt(3*i+1:3*i+3);
        xj = gt(3*(i+1)+1:3*(i+1)+3);
        z = se2_between(xi, xj);
        z = z + [0.05*randn; 0.05*randn; 0.02*randn];
        z(3) = wrap_angle(z(3));
        edges(end+1) = struct('i',i,'j',i+1,'z',z,'Omega',OmOdo); %#ok<AGROW>
    end

    OmLoop = make_information(0.03, 0.01);
    for i=0:10:(N-13)
        j = i + 10;
        xi = gt(3*i+1:3*i+3);
        xj = gt(3*j+1:3*j+3);
        z = se2_between(xi, xj);
        z = z + [0.03*randn; 0.03*randn; 0.01*randn];
        z(3) = wrap_angle(z(3));
        edges(end+1) = struct('i',i,'j',j,'z',z,'Omega',OmLoop); %#ok<AGROW>
    end

    % initial guess: chaining noisy odometry edges
    x0 = zeros(3*N,1);
    cur = [0;0;0];
    x0(1:3) = cur;
    for i=0:(N-2)
        z = edges(i+1).z; % odometry edge
        cur = se2_compose(cur, z);
        x0(3*(i+1)+1:3*(i+1)+3) = cur;
    end
end

function [posRMSE, angRMSE] = rmse_pose(x, gt)
    N = numel(x)/3;
    dp = zeros(N,1);
    da = zeros(N,1);
    for i=0:(N-1)
        p = x(3*i+1:3*i+2);
        pg = gt(3*i+1:3*i+2);
        dp(i+1) = norm(p - pg);
        da(i+1) = abs(wrap_angle(x(3*i+3) - gt(3*i+3)));
    end
    posRMSE = sqrt(mean(dp.^2));
    angRMSE = sqrt(mean(da.^2));
end

% -------------------- Optional Simulink model skeleton --------------------
function create_pose_graph_simulink_model()
    % Programmatic creation of a small Simulink model shell for iterative optimization.
    % This function is OPTIONAL and intended to show how Simulink can orchestrate
    % repeated optimization steps (e.g., in a larger estimation pipeline).
    %
    % The model uses a MATLAB Function block as the optimizer step.
    %
    % NOTE: This creates a model only; you still need to feed real edges/state.

    mdl = "Chapter12_Lesson5_PoseGraphSimulink";
    if bdIsLoaded(mdl), close_system(mdl,0); end
    new_system(mdl); open_system(mdl);

    add_block("simulink/Sources/Constant", mdl+"/x0", "Value", "zeros(180,1)");
    add_block("simulink/Sinks/To Workspace", mdl+"/x_out", "VariableName", "x_out");
    add_block("simulink/User-Defined Functions/MATLAB Function", mdl+"/GN_Step");

    set_param(mdl+"/GN_Step", "MATLABFcn", ...
        "function x_next = f(x)\n% one Gauss-Newton step placeholder\nx_next = x; % replace with real step\nend");

    add_line(mdl, "x0/1", "GN_Step/1");
    add_line(mdl, "GN_Step/1", "x_out/1");

    save_system(mdl);
    disp("Created Simulink model: " + mdl);
end
