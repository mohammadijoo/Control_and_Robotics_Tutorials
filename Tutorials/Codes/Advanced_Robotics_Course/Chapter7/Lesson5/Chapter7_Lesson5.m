function q = demo_grasp_score_matlab()
    % Example contacts
    contacts(1).p = [0.05; 0.0; 0.0];
    contacts(1).n = [-1.0; 0.0; 0.0];
    contacts(1).mu = 0.7;

    contacts(2).p = [-0.05; 0.04; 0.0];
    contacts(2).n = [1.0; -1.0; 0.0];
    contacts(2).mu = 0.7;

    contacts(3).p = [-0.05; -0.04; 0.0];
    contacts(3).n = [1.0; 1.0; 0.0];
    contacts(3).mu = 0.7;

    for i = 1:numel(contacts)
        contacts(i).n = contacts(i).n / norm(contacts(i).n);
    end

    W = build_wrench_set(contacts, 8, 1.0);
    q = epsilon_quality(W, 512);
    fprintf("Approx epsilon quality: %f\n", q);
end

function dirs = discretize_friction_cone(n, mu, k)
    nn = n / norm(n);
    tmp = [1.0; 0.0; 0.0];
    if abs(dot(tmp, nn)) > 0.9
        tmp = [0.0; 1.0; 0.0];
    end
    t1 = cross(nn, tmp);
    t1 = t1 / norm(t1);
    t2 = cross(nn, t1);

    dirs = zeros(3, k);
    for i = 1:k
        theta = 2.0 * pi * (i - 1) / k;
        t = cos(theta) * t1 + sin(theta) * t2;
        f = nn + mu * t;
        f = f / norm(f);
        dirs(:, i) = f;
    end
end

function W = build_wrench_set(contacts, k, wrench_scale)
    M = numel(contacts) * k;
    W = zeros(M, 6);
    row = 1;
    for idx = 1:numel(contacts)
        c = contacts(idx);
        dirs = discretize_friction_cone(c.n, c.mu, k);
        for j = 1:k
            f = wrench_scale * dirs(:, j);
            m = cross(c.p, f);
            W(row, 1:3) = f.';
            W(row, 4:6) = m.';
            row = row + 1;
        end
    end
end

function q = epsilon_quality(W, num_directions)
    if isempty(W)
        q = 0.0;
        return;
    end
    dim = 6;
    Nd = num_directions;
    % Sample Gaussian, then normalize rows
    X = randn(Nd, dim);
    norms = sqrt(sum(X.^2, 2));
    U = X ./ norms;
    proj = U * W.';  % Nd x M
    support_vals = max(proj, [], 2);
    q = min(support_vals);
end
      
