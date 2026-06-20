% Chapter10_Lesson2.m
% ICP Variants for Mobile Robots (2D SE(2) focus)
%
% Implements:
%   - Point-to-Point ICP (closed-form 2D Procrustes)
%   - Point-to-Line ICP (linearized least squares)
%   - Optional robust Huber weighting and trimming
%
% Notes:
%   - For 3D point clouds, Robotics System Toolbox offers pcregistericp.
%   - This script is self-contained (no toolboxes required), but uses O(N*M)
%     nearest-neighbor search for clarity.

function Chapter10_Lesson2()
    rng(2);

    % Synthetic "scan"
    M = 300;
    a = linspace(0, 2*pi, M)';
    dst = [2*cos(a) + 0.3*cos(5*a), 1*sin(a) + 0.2*sin(3*a)];

    trueTheta = 0.25;
    trueT = [0.8; -0.4];
    Rtrue = rot2(trueTheta);
    src = (Rtrue * dst')' + trueT';
    src = src + 0.01*randn(size(src));

    % Run point-to-line ICP (faster local convergence)
    opts.variant = "p2l";
    opts.maxIter = 60;
    opts.tol = 1e-7;
    opts.rejectDist = 0.5;
    opts.trimRatio = 0.9;      % keep 90% smallest correspondences
    opts.huberDelta = 0.05;    % optional
    [R,t,theta,history] = icp_se2(src, dst, opts);

    fprintf("Estimated theta = %.6f, t = [%.6f, %.6f]\n", theta, t(1), t(2));
    fprintf("True      theta = %.6f, t = [%.6f, %.6f]\n", trueTheta, trueT(1), trueT(2));

    % Plot
    srcAligned = (R*src')' + t';
    figure; hold on; axis equal;
    plot(dst(:,1), dst(:,2), '.', 'DisplayName', 'dst (reference)');
    plot(src(:,1), src(:,2), '.', 'DisplayName', 'src (raw)');
    plot(srcAligned(:,1), srcAligned(:,2), '.', 'DisplayName', 'src aligned');
    legend; title('ICP alignment (2D)');
end

function R = rot2(theta)
    c = cos(theta); s = sin(theta);
    R = [c -s; s c];
end

function n = estimate_normals_2d(dst)
    M = size(dst,1);
    prev = dst([M, 1:M-1], :);
    nxt  = dst([2:M, 1], :);
    tang = nxt - prev;
    n = [-tang(:,2), tang(:,1)];
    n = n ./ (sqrt(sum(n.^2,2)) + 1e-12);
end

function [dmin, idx] = nearest_bruteforce(P, Q)
    % P: Nx2, Q: Mx2
    N = size(P,1); M = size(Q,1);
    idx = zeros(N,1);
    dmin = zeros(N,1);
    for i=1:N
        dif = Q - P(i,:);
        dsq = sum(dif.^2,2);
        [m, j] = min(dsq);
        idx(i) = j;
        dmin(i) = sqrt(m);
    end
end

function w = huber_weights(r, delta)
    a = abs(r);
    w = ones(size(r));
    mask = a > delta;
    w(mask) = delta ./ (a(mask) + 1e-12);
end

function [R,t,theta,history] = icp_se2(src, dst, opts)
    arguments
        src (:,2) double
        dst (:,2) double
        opts.variant (1,1) string = "p2p"
        opts.maxIter (1,1) double = 40
        opts.tol (1,1) double = 1e-6
        opts.rejectDist (1,1) double = 0.0
        opts.huberDelta (1,1) double = 0.0
        opts.trimRatio (1,1) double = 0.0
    end

    R = rot2(0.0);
    t = [0.0; 0.0];

    if opts.variant == "p2l"
        dstNormals = estimate_normals_2d(dst);
    else
        dstNormals = [];
    end

    history.cost = [];
    history.inliers = [];

    prevCost = inf;

    for it=1:opts.maxIter
        srcT = (R*src')' + t';
        [d, idx] = nearest_bruteforce(srcT, dst);
        Q = dst(idx,:);

        mask = true(size(d));
        if opts.rejectDist > 0
            mask = mask & (d <= opts.rejectDist);
        end

        if opts.trimRatio > 0
            cand = find(mask);
            if ~isempty(cand)
                k = max(3, floor(opts.trimRatio * numel(cand)));
                [~,ord] = sort(d(cand));
                keep = cand(ord(1:k));
                mask(:) = false;
                mask(keep) = true;
            end
        end

        if nnz(mask) < 3
            break;
        end

        P = src(mask,:);
        Qm = Q(mask,:);

        if opts.variant == "p2p"
            % robust weights on Euclidean residual
            w = ones(size(P,1),1);
            if opts.huberDelta > 0
                E = (R*P')' + t' - Qm;
                rnorm = sqrt(sum(E.^2,2));
                w = huber_weights(rnorm, opts.huberDelta);
            end
            [R,t] = solve_p2p(P, Qm, w);
            E = (R*P')' + t' - Qm;
            cost = mean(sum(E.^2,2));
        else
            n = dstNormals(idx(mask), :);
            [Rdelta, tdelta, r] = solve_p2l(P, Qm, n, R, t, ones(size(P,1),1));
            % compose delta o current
            R = Rdelta * R;
            t = Rdelta * t + tdelta;

            if opts.huberDelta > 0
                % one IRLS step
                Pupd = (R*P')' + t';
                r2 = sum(n .* (Pupd - Qm), 2);
                w2 = huber_weights(r2, opts.huberDelta);
                [Rdelta, tdelta, ~] = solve_p2l(P, Qm, n, R, t, w2);
                R = Rdelta * R;
                t = Rdelta * t + tdelta;
                Pupd = (R*P')' + t';
                r2 = sum(n .* (Pupd - Qm), 2);
                cost = mean(r2.^2);
            else
                cost = mean(r.^2);
            end
        end

        history.cost(end+1) = cost;
        history.inliers(end+1) = nnz(mask);

        if abs(prevCost - cost) < opts.tol
            break;
        end
        prevCost = cost;
    end

    theta = atan2(R(2,1), R(1,1));
end

function [R,t] = solve_p2p(P, Q, w)
    w = w(:);
    wsum = sum(w) + 1e-12;
    pbar = (w' * P) / wsum;
    qbar = (w' * Q) / wsum;

    P0 = P - pbar;
    Q0 = Q - qbar;

    H = (P0' .* w') * Q0;
    [U,~,V] = svd(H);
    R = V*U';
    if det(R) < 0
        V(:,2) = -V(:,2);
        R = V*U';
    end
    t = qbar' - R*pbar';
end

function [Rdelta, tdelta, r] = solve_p2l(P, Q, n, R0, t0, w)
    % residual r = n^T (R0 p + t0 - q)
    p = (R0*P')' + t0';
    e = p - Q;
    r = sum(n .* e, 2);

    perp = [-P(:,2), P(:,1)];
    Jtheta = sum(n .* (R0*perp')', 2);
    A = [n, Jtheta];

    W = sqrt(w(:));
    Aw = A .* W;
    bw = r .* W;

    delta = Aw \ (-bw);
    dx = delta(1); dy = delta(2); dtheta = delta(3);

    Rdelta = rot2(dtheta);
    tdelta = [dx; dy];
end
