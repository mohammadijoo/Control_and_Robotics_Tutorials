function fc = isForceClosure2D(positions, normals, mu, eps)
% positions: N x 2 (x,y)
% normals:   N x 2 (nx,ny), inward unit normals
% mu:        scalar friction coefficient
% eps:       minimum alpha (interiority); default 1e-4

if nargin < 4
    eps = 1e-4;
end

N = size(positions, 1);
wcols = [];

for i = 1:N
    p = positions(i, :).';
    n = normals(i, :).';
    t = [-n(2); n(1)];
    phi = atan(mu);
    d1 = cos(phi) * n + sin(phi) * t;
    d2 = cos(phi) * n - sin(phi) * t;
    dirs = [d1, d2];
    for j = 1:2
        d = dirs(:, j);
        fx = d(1); fy = d(2);
        x = p(1); y = p(2);
        m = x * fy - y * fx;
        wcols = [wcols, [fx; fy; m]]; %#ok<AGROW>
    end
end

G = wcols;                % 3 x M
[~, s, ~] = svd(G);
rankG = sum(diag(s) > 1e-8);

if rankG < 3
    fc = false;
    return;
end

[m, M] = size(G);
Aeq = [G; ones(1, M)];
beq = [zeros(m, 1); 1];

f = zeros(M, 1);
lb = eps * ones(M, 1);
ub = [];  % no upper bounds

options = optimoptions("linprog", ...
    "Algorithm", "dual-simplex", ...
    "Display", "none");

[alpha, ~, exitflag] = linprog(f, [], [], Aeq, beq, lb, ub, options);
fc = (exitflag == 1);
end
      
