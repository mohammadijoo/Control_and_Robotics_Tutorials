function grasp_quality_demo()
    % Example contact geometry
    p_list = [ 0.05,    0.0,      0.0;
              -0.025,   0.043301, 0.0;
              -0.025,  -0.043301, 0.0 ];
    mu_list = [0.8, 0.8, 0.8];
    m = size(p_list,1);
    R_list = cell(m,1);

    for i = 1:m
        p = p_list(i,:).';
        n = -p / norm(p);
        tmp = [1; 0; 0];
        if abs(tmp.' * n) > 0.9
            tmp = [0; 1; 0];
        end
        x = tmp - (tmp.'*n)*n;
        x = x / norm(x);
        y = cross(n, x);
        R = [x, y, n];
        R_list{i} = R;
    end

    k_dirs = 8;
    W = primitive_wrenches(p_list, R_list, mu_list, k_dirs);

    % Isotropy metric via SVD of W
    s = svd(W);
    q_iso = min(s) / max(s);

    % Sampled epsilon metric
    q_eps = epsilon_sampled(W, 2000);

    fprintf('Isotropy metric Q_iso = %.4f\n', q_iso);
    fprintf('Approx epsilon metric Q_eps = %.4f\n', q_eps);
end

function S = skew(p)
    px = p(1); py = p(2); pz = p(3);
    S = [  0,  -pz,  py;
          pz,   0,  -px;
         -py,  px,   0 ];
end

function G_i = grasp_map_block(p_i, R_i)
    upper = R_i;
    lower = skew(p_i) * R_i;
    G_i = [upper; lower];
end

function D = linearized_friction_directions(mu, k_dirs)
    alpha = atan(mu);
    fn = cos(alpha);
    ft = sin(alpha);
    D = zeros(3, k_dirs);
    for j = 1:k_dirs
        theta = 2*pi*(j-1)/k_dirs;
        tx = ft * cos(theta);
        ty = ft * sin(theta);
        D(:,j) = [tx; ty; fn];
    end
end

function W = primitive_wrenches(p_list, R_list, mu_list, k_dirs)
    m = size(p_list,1);
    N = m * k_dirs;
    W = zeros(6, N);
    col = 1;
    for i = 1:m
        p_i = p_list(i,:).';
        R_i = R_list{i};
        G_i = grasp_map_block(p_i, R_i);
        D_i = linearized_friction_directions(mu_list(i), k_dirs);
        W_i = G_i * D_i;
        W(:, col:col+k_dirs-1) = W_i;
        col = col + k_dirs;
    end
end

function q_eps = epsilon_sampled(W, n_directions)
    % Normalize primitive wrenches to form a "unit" polytope
    norms = sqrt(sum(W.^2, 1));
    Wn = W ./ max(norms, 1e-12);

    q_eps = inf;
    for k = 1:n_directions
        v = randn(6,1);
        v = v / norm(v);
        support = max(v.' * Wn);
        q_eps = min(q_eps, support);
    end
end
      
